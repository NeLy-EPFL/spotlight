"""Loads `TinyLocalizationModel` training examples from cached, pre-resized
raw (unaligned, uncropped) camera frames (see
`scripts/postprocessing/model_training/localization/cache_fullsize_frames.py`), deriving labels from
an existing RepVGG-A0 model's exhaustive per-frame predictions (see
`spotlight.postprocessing.pose2d`) rather than any hand labels.

For each sampled frame: neck/thorax/abdomen positions come from that
model's predicted `N`/`Th`/`A` keypoints, in its own aligned (cropped)
domain; that trial's own per-frame alignment transform (inverted) maps them
back to the raw fullsize camera frame, then a further 1/`SCALE_FACTOR` puts
them in this tiny model's own coordinate space, matching the resizing
`cache_fullsize_frames.py` applied to the raw frame itself. The `flipped`
target comes from `flip_label.is_flipped` applied to
`flip_label.weighted_confidence`, a proxy derived from that same model's
own per-keypoint confidence; hand-labeled/hand-corrected frames have no
confidence value at all, so `valid_frames` excludes them from every
trial's candidate pool entirely (they never make it into the dataset).
"""

from pathlib import Path

import cv2
import h5py
import numpy as np
import torch
from loguru import logger
from torch.utils.data import Dataset
from tqdm import tqdm

from spotlight.postprocessing.localization.augment import augment
from spotlight.postprocessing.localization.constants import (
    KEYPOINT_SPEC,
    OUTPUT_SIZE,
    SCALE_FACTOR,
)
from spotlight.postprocessing.localization.flip_label import (
    is_flipped,
    valid_frames,
    weighted_confidence,
)
from spotlight.postprocessing.pose2d.geometry import (
    apply_affine,
    invert_affine,
)
from spotlight.postprocessing.pose2d.io_utils import (
    load_pose_h5,
    parse_trial_identity,
    trial_dir_from_video_path,
)

FULLSIZE_VIDEO_RELPATH = Path("postprocessed/fullsize_behavior_video.mkv")
# Same file/dataset `scripts/postprocessing/model_training/pose2d/slp_apply_alignment.py` reads.
ALIGNMENT_TRANSFORMS_RELPATH = Path("postprocessed/behavior_alignment_transforms.h5")


def coarse_points_aligned_domain(
    poses: np.ndarray, node_names: list[str]
) -> np.ndarray:
    """`(n_frames, 3, 2)` neck/thorax/abdomen (see `KEYPOINT_SPEC`), in the
    RepVGG-A0 model's own aligned-domain coordinate scale (900x900,
    `pose2d.dataset.ALIGNED_FRAME_SIZE`).
    """
    index = {name: i for i, name in enumerate(node_names)}
    points = [
        np.mean([poses[:, index[node]] for node in nodes], axis=0)
        for nodes in KEYPOINT_SPEC.values()
    ]
    return np.stack(points, axis=1)


def load_alignment_transforms(trial_dir: Path) -> np.ndarray:
    """This trial's per-frame raw-to-aligned affine transforms.

    Args:
        trial_dir: Trial directory containing `ALIGNMENT_TRANSFORMS_RELPATH`.

    Returns:
        `(n_frames, 2, 3)` affine matrices, one per frame.
    """
    with h5py.File(trial_dir / ALIGNMENT_TRANSFORMS_RELPATH, "r") as f:
        return f["transform_matrices"][:]


def points_to_model_space(
    aligned_points: np.ndarray,
    transform_matrices: np.ndarray,
    output_size: tuple[int, int],
) -> np.ndarray:
    """Maps aligned-domain points back to this model's own raw-frame,
    downscaled, `[0, 1]`-normalized coordinate space.

    Args:
        aligned_points: `(n_frames, n_points, 2)`, in the 900x900 aligned domain.
        transform_matrices: `(n_frames, 2, 3)` raw-to-aligned affine
            transforms, one per frame (same frame order as `aligned_points`).
        output_size: `(width, height)` this model's own resized input frames
            are scaled to.

    Returns:
        `(n_frames, n_points, 2)`, normalized to `[0, 1]` by `output_size`.
    """
    raw_points = apply_affine(aligned_points, invert_affine(transform_matrices))
    model_points = raw_points / SCALE_FACTOR
    return model_points / np.array(output_size, dtype=np.float32)


def load_cached_frame(frame_path: Path) -> np.ndarray:
    """Loads one `cache_fullsize_frames.py`-cached JPEG as `(H, W, 3)` uint8
    (repeating its single grayscale channel 3x, to match the RGB-shaped
    input `TinyLocalizationModel` expects).
    """
    image = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise SystemExit(
            f"Missing cached frame {frame_path}; run "
            "scripts/postprocessing/model_training/localization/cache_fullsize_frames.py first."
        )
    if image.ndim == 3:
        image = image[:, :, 0]
    return np.repeat(image[:, :, np.newaxis], 3, axis=-1)


class TinyLocalizationDataset(Dataset):
    """One example per sampled frame: a resized raw camera frame, its
    coarse keypoint target (see `KeypointSpec`), and a binary "flipped"
    target (see module docstring).
    """

    def __init__(
        self,
        h5_paths: list[Path],
        frame_cache_root: Path,
        train: bool,
        output_size: tuple[int, int] = OUTPUT_SIZE,
        max_frames_per_trial: int | None = None,
        seed: int = 0,
    ) -> None:
        self.output_size = output_size
        self.train = train
        self.rng = np.random.default_rng(seed)
        rng = self.rng
        width, height = output_size

        all_images, all_keypoints, all_flipped = [], [], []
        progress = tqdm(h5_paths, desc="Loading trials", mininterval=1.0)
        for h5_path in progress:
            data = load_pose_h5(h5_path)
            trial_dir = trial_dir_from_video_path(data["video_path"])
            genotype, fly_trial = parse_trial_identity(data["video_path"])
            progress.set_postfix_str(f"{genotype}__{fly_trial}")

            all_confidence = weighted_confidence(
                data["keypoint_scores"], data["node_names"]
            )
            candidate_idxs, confidence = valid_frames(
                data["keypoint_scores"],
                data["node_names"],
                data["is_label"],
                confidence=all_confidence,
            )
            if max_frames_per_trial is not None and max_frames_per_trial < len(
                candidate_idxs
            ):
                chosen = np.sort(
                    rng.choice(
                        len(candidate_idxs), size=max_frames_per_trial, replace=False
                    )
                )
                frame_idxs, confidence = candidate_idxs[chosen], confidence[chosen]
            else:
                frame_idxs = candidate_idxs

            aligned_points = coarse_points_aligned_domain(
                data["poses"][frame_idxs], data["node_names"]
            )
            transform_matrices = load_alignment_transforms(trial_dir)[frame_idxs]
            keypoints = points_to_model_space(
                aligned_points, transform_matrices, output_size
            )
            flipped = is_flipped(confidence)

            frame_dir = (
                frame_cache_root / f"{genotype}__{fly_trial}" / f"{width}x{height}"
            )
            images = np.stack(
                [
                    load_cached_frame(frame_dir / f"frame_{idx:09d}.jpg")
                    for idx in frame_idxs
                ]
            )

            all_images.append(images)
            all_keypoints.append(keypoints)
            all_flipped.append(flipped)
            logger.info(
                f"{genotype}__{fly_trial}: loaded {len(frame_idxs)} frames "
                f"({flipped.sum()} flipped)"
            )

        self.images = np.concatenate(all_images)
        self.keypoints = np.concatenate(all_keypoints).astype(np.float32)
        self.flipped = np.concatenate(all_flipped).astype(np.float32)
        logger.info(f"Loaded {len(self.images)} frames from {len(h5_paths)} trial(s)")

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        image = self.images[idx]
        keypoints = self.keypoints[idx]
        if self.train:
            image, keypoints = augment(image, keypoints, self.rng)

        image = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        keypoints = torch.from_numpy(keypoints)
        flipped = torch.tensor([self.flipped[idx]], dtype=torch.float32)
        return image, keypoints, flipped
