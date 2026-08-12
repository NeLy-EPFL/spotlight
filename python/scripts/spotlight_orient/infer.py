#!/usr/bin/env python
"""Runs a trained `TinyOrientModel` checkpoint over every cached frame of
one trial (see `scripts/spotlight_orient/cache_fullsize_frames.py`),
predicting head/thorax/abdomen position and flip probability for each.

Reads cached frames directly, not the source video -- that trial must
already be cached at `output_size` first. Predictions are saved densely
(one row per raw video frame, NaN for any frame missing from the cache),
with keypoints converted back to the raw fullsize camera frame's own pixel
domain (`dataset.NATIVE_FRAME_SIZE`), the natural coordinate space for
downstream use (e.g. driving an alignment/crop step).

Usage:
    python tools/spotlight_orient/infer.py \\
        --checkpoint-path bulk_data/.../orient_model/checkpoints/v1/best.pt \\
        --input-path bulk_data/.../final_predictions/<trial>_pose2d_final_predictions.h5 \\
        --frame-cache-root bulk_data/motion_prior/orient_model/frame_cache \\
        --output-h5-path bulk_data/.../orient_model/predictions/<trial>_orient_predictions.h5
"""

import re
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import tyro
from loguru import logger
from tqdm import tqdm

from spotlight_postprocessing.spotlight_orient.dataset import (
    COARSE_KEYPOINTS,
    FULLSIZE_VIDEO_RELPATH,
    OUTPUT_SIZE,
    SCALE_FACTOR,
)
from spotlight_postprocessing.spotlight_orient.io_utils import (
    save_orient_predictions_h5,
)
from spotlight_postprocessing.spotlight_orient.model import TinyOrientModel
from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    parse_trial_identity,
    trial_dir_from_video_path,
)

FRAME_FILENAME_PATTERN = re.compile(r"frame_(\d+)\.jpg")


def cached_frame_paths(frame_dir: Path) -> list[tuple[int, Path]]:
    """Every cached frame under `frame_dir`, as sorted `(frame_idx, path)` pairs."""
    paths = sorted(frame_dir.glob("frame_*.jpg"))
    if not paths:
        raise SystemExit(
            f"No cached frames found under {frame_dir}; run "
            "scripts/spotlight_orient/cache_fullsize_frames.py first."
        )
    return [(int(FRAME_FILENAME_PATTERN.match(p.name)[1]), p) for p in paths]


def load_batch(paths: list[Path]) -> np.ndarray:
    """`(batch, H, W, 3)` uint8, repeating each cached grayscale JPEG's
    single channel 3x to match `TinyOrientModel`'s RGB-shaped input.
    """
    images = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image.ndim == 3:
            image = image[:, :, 0]
        images.append(image)
    return np.repeat(np.stack(images)[:, :, :, None], 3, axis=-1)


def main(
    checkpoint_path: Path,
    input_path: Path,
    frame_cache_root: Path,
    output_h5_path: Path,
    output_size: tuple[int, int] = OUTPUT_SIZE,
    use_global_context: bool = True,
    batch_size: int = 256,
    override: bool = False,
) -> None:
    """Predict every cached frame of one trial.

    Args:
        checkpoint_path: Trained `TinyOrientModel` state dict.
        input_path: This trial's `final_predictions.h5` (only its
            `video_path` attr is used, to identify the trial).
        frame_cache_root: Root directory
            `scripts/spotlight_orient/cache_fullsize_frames.py` wrote into.
        output_h5_path: Where to save predictions (see `io_utils.save_orient_predictions_h5`).
            Aborts if this already exists, unless `override` is set.
        output_size: `(width, height)` cached frames were resized to; see
            `dataset.OUTPUT_SIZE`.
        use_global_context: Must match what `checkpoint_path` was actually
            trained with -- False for v1-v5 checkpoints, True from v6 on
            (see `model.GlobalContextBlock`/`train.py`'s own flag).
        batch_size: Frames per inference batch.
        override: If True, overwrite `output_h5_path` if it already exists.
    """
    check_output_path(output_h5_path, override)
    keypoint_names = COARSE_KEYPOINTS
    trial_data = load_pose_h5(input_path)
    trial_dir = trial_dir_from_video_path(trial_data["video_path"])
    genotype, fly_trial = parse_trial_identity(trial_data["video_path"])
    raw_video_path = trial_dir / FULLSIZE_VIDEO_RELPATH
    logger.info(f"Trial: {genotype}__{fly_trial}")
    logger.info(f"Keypoints: {keypoint_names}")

    width, height = output_size
    frame_dir = frame_cache_root / f"{genotype}__{fly_trial}" / f"{width}x{height}"
    indexed_paths = cached_frame_paths(frame_dir)
    n_frames = max(idx for idx, _ in indexed_paths) + 1

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = TinyOrientModel(
        n_keypoints=len(keypoint_names), use_global_context=use_global_context
    ).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    logger.info(f"Device: {device}")

    keypoints = np.full((n_frames, len(keypoint_names), 2), np.nan, dtype=np.float32)
    flipped_prob = np.full(n_frames, np.nan, dtype=np.float32)

    starts = list(range(0, len(indexed_paths), batch_size))
    is_tty = sys.stdout.isatty()
    log_every = max(1, int(0.01 * len(starts)))
    iterator = tqdm(starts, mininterval=1.0) if is_tty else starts

    with torch.no_grad():
        for batch_idx, start in enumerate(iterator):
            batch = indexed_paths[start : start + batch_size]
            frame_idxs = [idx for idx, _ in batch]
            batch_images = load_batch([p for _, p in batch])
            batch_tensor = (
                torch.from_numpy(batch_images).permute(0, 3, 1, 2).float().to(device)
                / 255.0
            )

            with torch.autocast(
                device_type=device, dtype=torch.float16, enabled=device == "cuda"
            ):
                pred_keypoints, pred_flip_logit = model(batch_tensor)

            # Sigmoid-normalized [0, 1] model space -> cached-frame pixel
            # space -> raw fullsize pixel space (see dataset.points_to_model_space).
            raw_points = (
                pred_keypoints.float().cpu().numpy()
                * np.array(output_size, dtype=np.float32)
                * SCALE_FACTOR
            )
            probs = torch.sigmoid(pred_flip_logit).float().cpu().numpy()[:, 0]

            for offset, frame_idx in enumerate(frame_idxs):
                keypoints[frame_idx] = raw_points[offset]
                flipped_prob[frame_idx] = probs[offset]

            if not is_tty and batch_idx % log_every == 0:
                logger.info(f"{start}/{len(indexed_paths)} frames")

    save_orient_predictions_h5(
        output_h5_path, keypoints, flipped_prob, keypoint_names, raw_video_path
    )
    logger.info(f"Predicted {len(indexed_paths)}/{n_frames} frames -> {output_h5_path}")


if __name__ == "__main__":
    tyro.cli(main)
