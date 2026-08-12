#!/usr/bin/env python
"""Runs a trained pose2d checkpoint over every frame of one aligned-domain
video, producing a dense pose `.h5` (see
`spotlight_postprocessing.pose2d.io_utils`) with `is_label` all
False, so it can be ported into a `.slp` for GUI correction via
`slp_convert_h5.py --h5-to-slp`.

Usage:
    python tools/spotlight_pose2d/infer.py \\
        --checkpoint-path bulk_data/.../best.pt \\
        --video-path .../processed/aligned_behavior_video.mkv \\
        --skeleton-json-path <JSON file from extract_metadata_from_initial_slp.py> \\
        --output-path predictions.h5
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import sleap_io as sio
import torch
import tyro
from loguru import logger
from tqdm import tqdm

from spotlight_postprocessing.pose2d.dataset import INPUT_SIZE
from spotlight_postprocessing.pose2d.io_utils import (
    check_output_path,
    load_skeleton_json,
    save_pose_h5,
)
from spotlight_postprocessing.pose2d.model import RepVGGPoseModel


def heatmaps_to_points(heatmaps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-keypoint argmax location and peak value, in heatmap pixel coords.

    Args:
        heatmaps: `(n_nodes, H, W)`.

    Returns:
        points: `(n_nodes, 2)` (x, y). scores: `(n_nodes,)`.
    """
    n_nodes, h, w = heatmaps.shape
    flat = heatmaps.reshape(n_nodes, -1)
    idx = flat.argmax(axis=1)
    scores = flat[np.arange(n_nodes), idx]
    ys, xs = np.unravel_index(idx, (h, w))
    return np.stack([xs, ys], axis=1).astype(np.float32), scores.astype(np.float32)


def main(
    checkpoint_path: Path,
    video_path: Path,
    skeleton_json_path: Path,
    output_path: Path,
    n_keypoints: int = 37,
    batch_size: int = 32,
    override: bool = False,
) -> None:
    """Run inference on every frame of `video_path`.

    Args:
        checkpoint_path: Trained `RepVGGPoseModel` state dict (`train.py`'s
            `best.pt`/`last.pt`).
        video_path: Aligned-domain video to run inference on.
        skeleton_json_path: JSON file with the target skeleton, from
            `extract_metadata_from_initial_slp.py` (only its node
            names/order are used).
        output_path: Where to save the dense pose `.h5`. Aborts if this
            already exists, unless `override` is set.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        batch_size: Frames per inference batch.
        override: If True, overwrite `output_path` if it already exists.
    """
    check_output_path(output_path, override)
    logger.info(f"Checkpoint: {checkpoint_path}")
    logger.info(f"Video: {video_path}")
    logger.info(f"Skeleton JSON: {skeleton_json_path}")
    logger.info(f"Output: {output_path}")

    node_names = [n.name for n in load_skeleton_json(skeleton_json_path).nodes]
    if len(node_names) != n_keypoints:
        raise SystemExit(
            f"{skeleton_json_path} has {len(node_names)} nodes, expected {n_keypoints}"
        )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()

    video = sio.Video(filename=str(video_path))
    n_frames, _height, width, _channels = video.shape
    input_scale = INPUT_SIZE / width
    logger.info(f"{n_frames} frames, device={device}")

    poses = np.full((n_frames, n_keypoints, 2), np.nan, dtype=np.float32)
    keypoint_scores = np.full((n_frames, n_keypoints), np.nan, dtype=np.float32)
    instance_score = np.full(n_frames, np.nan, dtype=np.float32)

    starts = range(0, n_frames, batch_size)
    n_batches = len(starts)
    log_every = max(1, int(0.01 * n_batches))
    is_tty = sys.stdout.isatty()
    iterator = tqdm(starts, mininterval=1.0) if is_tty else starts

    with torch.no_grad():
        for batch_idx, start in enumerate(iterator):
            end = min(start + batch_size, n_frames)
            batch = np.stack([video[i] for i in range(start, end)])
            if batch.shape[-1] == 1:
                batch = np.repeat(batch, 3, axis=-1)
            batch = np.stack([cv2.resize(f, (INPUT_SIZE, INPUT_SIZE)) for f in batch])
            tensor = (
                torch.from_numpy(batch).permute(0, 3, 1, 2).float().to(device) / 255.0
            )

            heatmaps = model(tensor).cpu().numpy()
            heatmap_scale = INPUT_SIZE / heatmaps.shape[-1] / input_scale
            for offset, heatmap in enumerate(heatmaps):
                points, scores = heatmaps_to_points(heatmap)
                poses[start + offset] = points * heatmap_scale
                keypoint_scores[start + offset] = scores
                instance_score[start + offset] = scores.mean()

            if not is_tty and batch_idx % log_every == 0:
                logger.info(f"{start}/{n_frames} frames")

    save_pose_h5(
        output_path, poses, keypoint_scores, instance_score,
        np.zeros(n_frames, dtype=bool), node_names, video_path,
    )  # fmt: skip
    logger.info(f"Saved {n_frames} frames -> {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
