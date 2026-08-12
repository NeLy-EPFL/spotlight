"""Loads pose2d training examples: pre-cached aligned-domain video frames +
their label keypoints, with optional augmentation.

Reads one or more per-trial dense pose `.h5` files (see
`spotlight_tools.spotlight_pose2d.io_utils.save_pose_h5`, produced by
`slp_convert_h5.py --slp-to-h5`), keeping only rows where `is_label` is
True, since only labeled frames have ground truth and the rest of a
trial's video is unused for training. Frame images come from
`cache_video_frames.py`'s JPEG cache (see `cache_all_trial_frames.py`),
already resized to `input_size`, keyed by the trial's genotype/fly_trial
(parsed from the `.h5`'s own `video_path` attr, not read from the source
video itself, which decoding/seeking into repeatedly was measured far
slower).
"""

from pathlib import Path

import cv2
import numpy as np
import torch
from loguru import logger
from torch.utils.data import Dataset

from spotlight_tools.spotlight_pose2d.augment import augment
from spotlight_tools.spotlight_pose2d.io_utils import (
    load_pose_h5,
    parse_trial_identity,
)

INPUT_SIZE = 450
# Every aligned-domain video this project produces is cropped/warped to
# this size; the `.h5`'s own stored points are in this native scale, not
# `INPUT_SIZE`, since alignment and JPEG-caching are independent steps.
ALIGNED_FRAME_SIZE = 900
HEATMAP_SIGMA = 2.0


def cached_labeled_frames(
    h5_path: Path, frame_cache_root: Path, input_size: int
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """This trial's labeled frame images, keypoints, and node names.

    Args:
        h5_path: This trial's dense pose `.h5` (see `save_pose_h5`).
        frame_cache_root: Root directory `cache_video_frames.py` wrote into
            (one subdirectory per trial, one per size within that; see
            `caller_scripts/cache_all_trial_frames.py`).
        input_size: Resolution the frame cache was built at; must match an
            existing `<input_size>x<input_size>` subdirectory per trial.

    Returns:
        images: `(n_labeled, input_size, input_size)` uint8.
        points: `(n_labeled, n_nodes, 2)` float32, rescaled from the `.h5`'s
            native `ALIGNED_FRAME_SIZE` scale to `input_size`.
        node_names: Skeleton node name per column of `points`.
    """
    data = load_pose_h5(h5_path)
    label_idxs = np.flatnonzero(data["is_label"])
    points = data["poses"][label_idxs] * (input_size / ALIGNED_FRAME_SIZE)

    genotype, fly_trial = parse_trial_identity(data["video_path"])
    frame_dir = (
        frame_cache_root / f"{genotype}__{fly_trial}" / f"{input_size}x{input_size}"
    )

    images = []
    for idx in label_idxs:
        frame_path = frame_dir / f"frame_{idx:09d}.jpg"
        image = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise SystemExit(
                f"Missing cached frame {frame_path}; run "
                "caller_scripts/cache_all_trial_frames.py first."
            )
        images.append(image)
    return np.stack(images), points, data["node_names"]


def mirrored_node_name(name: str) -> str:
    """`name`'s left/right counterpart (e.g. `LF_ThC` <-> `RF_ThC`), found
    by swapping a leading `L`/`R`, or `name` unchanged for an unpaired
    midline node (e.g. `Th`, `N`, `A`).
    """
    if name.startswith("L"):
        return "R" + name[1:]
    if name.startswith("R"):
        return "L" + name[1:]
    return name


def flip_pair_indices(node_names: list[str]) -> np.ndarray:
    """`(n_nodes,)`: node i's left/right mirror index (itself, if
    unpaired), for relabeling keypoints after a horizontal flip -- see
    `augment.augment`.
    """
    index_by_name = {name: i for i, name in enumerate(node_names)}
    indices = []
    for name in node_names:
        mirror = mirrored_node_name(name)
        if mirror not in index_by_name:
            raise SystemExit(
                f"No left/right mirror node found for {name!r} (expected "
                f"{mirror!r} among {node_names})"
            )
        indices.append(index_by_name[mirror])
    return np.array(indices)


def points_to_heatmaps(points: np.ndarray, size: int, sigma: float) -> np.ndarray:
    """Gaussian heatmaps for `(n_nodes, 2)` points, in `size`x`size` output
    coordinates (NaN point -> all-zero heatmap for that keypoint).
    """
    n_nodes = points.shape[0]
    heatmaps = np.zeros((n_nodes, size, size), dtype=np.float32)
    yy, xx = np.mgrid[0:size, 0:size]
    for i, (x, y) in enumerate(points):
        if np.isnan(x) or np.isnan(y):
            continue
        heatmaps[i] = np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * sigma**2))
    return heatmaps


class PoseDataset(Dataset):
    """Concatenates labeled frames across one or more trials' `.h5` files."""

    def __init__(
        self,
        h5_paths: list[Path],
        frame_cache_root: Path,
        train: bool,
        heatmap_size: int,
        input_size: int = INPUT_SIZE,
        heatmap_sigma: float = HEATMAP_SIGMA,
        seed: int = 0,
    ) -> None:
        self.train = train
        self.input_size = input_size
        self.heatmap_size = heatmap_size
        self.heatmap_sigma = heatmap_sigma
        self.rng = np.random.default_rng(seed)

        all_images, all_points = [], []
        node_names = None
        for h5_path in h5_paths:
            images, points, trial_node_names = cached_labeled_frames(
                h5_path, frame_cache_root, input_size
            )
            if node_names is None:
                node_names = trial_node_names
            elif trial_node_names != node_names:
                raise SystemExit(
                    f"{h5_path} has different node names/order than earlier "
                    f"trials ({trial_node_names} != {node_names})"
                )
            all_images.append(images)
            all_points.append(points)
        self.images = np.concatenate(all_images)
        self.points = np.concatenate(all_points)
        self.flip_pair_indices = flip_pair_indices(node_names)
        logger.info(
            f"Loaded {len(self.images)} labeled frames from {len(h5_paths)} trial(s)"
        )

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        image = self.images[idx]
        points = self.points[idx].copy()

        if self.train:
            image, points = augment(image, points, self.rng, self.flip_pair_indices)
        if image.ndim == 3:
            image = image[:, :, 0]
        image = np.repeat(image[:, :, np.newaxis], 3, axis=-1)

        heatmap_points = points * (self.heatmap_size / self.input_size)
        heatmaps = points_to_heatmaps(
            heatmap_points, self.heatmap_size, self.heatmap_sigma
        )

        image_tensor = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        return image_tensor, torch.from_numpy(heatmaps)
