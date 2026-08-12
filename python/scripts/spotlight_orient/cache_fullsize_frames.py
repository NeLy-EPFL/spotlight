#!/usr/bin/env python
"""Decodes each trial's raw fullsize video once, sequentially, and caches
every frame resized 0.25x (`TinyOrientModel`'s own input resolution -- see
`spotlight_postprocessing.spotlight_orient.dataset.OUTPUT_SIZE`) as JPEG.

Same rationale as `spotlight_pose2d/cache_video_frames.py`: `TinyOrientDataset`
otherwise reads frames via `pvio`'s random-access seeking, measured at
~0.7 sec/frame (no cache for the raw fullsize video existed until now,
unlike spotlight_pose2d's own 450x450 aligned-domain cache); sequential
decode is far faster and only needs to happen once.

Trials come from `labels/final_predictions/*.h5`'s own `video_path` attr
(that trial's aligned video; `FULLSIZE_VIDEO_RELPATH` swaps it for the
sibling raw video) -- the same 29 trials `TinyOrientDataset` draws from,
independent of any particular train/val split, so every future training
round reuses this cache without touching the NAS again.

QUALITY matches `spotlight_pose2d/cache_all_trial_frames.py`'s own (90).

Runs N_WORKERS trials concurrently via joblib, same rationale as
`spotlight_pose2d/cache_all_trial_frames.py`: measured CPU-bound, not
NAS-bandwidth-bound. Two progress bar levels: one over trials, and one per
trial over its own frames (each concurrent worker gets its own terminal row).

Hardcoded constants, no CLI args.

Usage:
    python scripts/spotlight_orient/cache_fullsize_frames.py
"""

import multiprocessing
from pathlib import Path

import cv2
import numpy as np
import sleap_io as sio
from joblib import Parallel, delayed
from loguru import logger
from tqdm import tqdm

from spotlight_postprocessing.spotlight_orient.dataset import (
    FULLSIZE_VIDEO_RELPATH,
    OUTPUT_SIZE,
)
from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    load_pose_h5,
    parse_trial_identity,
    trial_dir_from_video_path,
)

N_WORKERS = 4
QUALITY = 90  # Matches spotlight_pose2d/cache_all_trial_frames.py.

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/final_predictions"
CACHE_ROOT = REPO_ROOT / "bulk_data/motion_prior/orient_model/frame_cache"


def to_2d(frame: np.ndarray) -> np.ndarray:
    """Squeezes a `(H, W, 1)` grayscale frame to `(H, W)`; passes color through."""
    if frame.ndim == 3 and frame.shape[-1] == 1:
        return frame[:, :, 0]
    return frame


def worker_slot() -> int:
    """This joblib/loky worker process's 0-indexed slot, stable for its
    lifetime -- lets each of the N_WORKERS concurrent per-trial progress
    bars claim its own terminal row (via tqdm's `position`) instead of all
    of them clobbering the same line. Falls back to 0 outside a worker
    process (e.g. N_WORKERS=1, running inline).
    """
    identity = multiprocessing.current_process()._identity
    return identity[0] - 1 if identity else 0


def process_trial(h5_path: Path) -> str:
    """Caches one trial's frames.

    Returns:
        The trial's stem (`<genotype>__<fly_trial>`).
    """
    data = load_pose_h5(h5_path)
    trial_dir = trial_dir_from_video_path(data["video_path"])
    genotype, fly_trial = parse_trial_identity(data["video_path"])
    stem = f"{genotype}__{fly_trial}"
    width, height = OUTPUT_SIZE
    output_dir = CACHE_ROOT / stem / f"{width}x{height}"

    if output_dir.is_dir() and any(output_dir.iterdir()):
        logger.info(f"Skipping {stem} (already cached)")
        return stem

    video_path = trial_dir / FULLSIZE_VIDEO_RELPATH
    output_dir.mkdir(parents=True, exist_ok=True)
    jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, QUALITY]

    video = sio.Video(filename=str(video_path))
    n_frames = video.shape[0]
    frame_idxs = tqdm(
        range(n_frames),
        desc=stem,
        position=worker_slot() + 1,
        leave=False,
        mininterval=1.0,
    )
    for frame_idx in frame_idxs:
        frame = to_2d(video[frame_idx])
        resized = cv2.resize(frame, OUTPUT_SIZE, interpolation=cv2.INTER_AREA)
        frame_path = output_dir / f"frame_{frame_idx:09d}.jpg"
        cv2.imwrite(str(frame_path), resized, jpeg_params)
    return stem


def main() -> None:
    h5_paths = sorted(DATA_ROOT.glob("*_pose2d_final_predictions.h5"))
    if not h5_paths:
        raise SystemExit(
            f"No *_pose2d_final_predictions.h5 files found under {DATA_ROOT}"
        )

    results = Parallel(n_jobs=N_WORKERS)(
        delayed(process_trial)(p) for p in tqdm(h5_paths, desc="Trials", position=0)
    )
    logger.info(f"Processed {len(results)} trials")


if __name__ == "__main__":
    main()
