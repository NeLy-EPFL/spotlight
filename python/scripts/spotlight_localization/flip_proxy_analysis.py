#!/usr/bin/env python
"""Validates `flip_label.proximal_leg_confidence` as an "is the fly flipped"
proxy: plots its distribution over every trial's `final_predictions.h5`
(excluding hand-labeled/hand-corrected frames, which have no confidence at
all -- see `flip_label.py`'s docstring), then samples both sides of
`FLIPPED_THRESHOLD` for visual inspection.

Saves, all under `OUTPUT_DIR`:
- `flip_confidence.npz`: each trial's per-frame confidence and the frame
  indices they correspond to (hand-labeled frames excluded), for reuse
  without re-reading every `.h5`.
- `flip_confidence_pooled_hist.png`: one histogram over every trial pooled.
- `flip_confidence_per_trial_hist.png`: a small-multiples grid, one
  histogram per trial -- checks whether the two modes (if any) sit at a
  consistent location across recording sessions, rather than pooling and
  hoping session-to-session confidence drift doesn't smear them.
- `flipped_samples/`, `unflipped_samples/`: N_SAMPLES_PER_CASE aligned-domain
  crops each, randomly sampled from below/above `FLIPPED_THRESHOLD`
  (pooled across trials), for a human to eyeball.

Hardcoded constants, no CLI args.

Usage:
    python scripts/spotlight_localization/flip_proxy_analysis.py
"""

from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pvio
from loguru import logger
from tqdm import tqdm

from spotlight_postprocessing.spotlight_localization.flip_label import (
    FLIPPED_THRESHOLD,
    is_flipped,
    valid_frames,
)
from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    load_pose_h5,
    parse_trial_identity,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/final_predictions"
OUTPUT_DIR = REPO_ROOT / "bulk_data/motion_prior/localization_model/flip_proxy_analysis"
N_BINS = 100
N_SAMPLES_PER_CASE = 1000
SAMPLE_SIZE = (225, 225)  # (width, height) -- 0.25x of the 900x900 aligned video.
JPEG_QUALITY = 90
SEED = 0


def load_trial_confidence(h5_path: Path) -> tuple[np.ndarray, np.ndarray, Path, int]:
    """One trial's per-frame confidence, excluding hand-labeled/
    hand-corrected frames (see module docstring).

    Returns:
        frame_idxs: Frame indices `confidence` corresponds to (a subset of
            `0..n_frames-1`).
        confidence: `proximal_leg_confidence`, one value per `frame_idxs` entry.
        video_path: This trial's aligned-domain video (from the `.h5`'s own
            `video_path` attr).
        n_hand: Number of hand-labeled/hand-corrected frames excluded.
    """
    data = load_pose_h5(h5_path)
    frame_idxs, confidence = valid_frames(
        data["keypoint_scores"], data["node_names"], data["is_label"]
    )
    return frame_idxs, confidence, data["video_path"], int(data["is_label"].sum())


def plot_pooled_histogram(
    all_values: np.ndarray, n_trials: int, output_path: Path
) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(all_values, bins=N_BINS, range=(0, 1), color="tab:blue")
    ax.axvline(
        FLIPPED_THRESHOLD,
        color="tab:red",
        linestyle="--",
        label=f"threshold={FLIPPED_THRESHOLD}",
    )
    ax.set_xlabel("Mean confidence over proximal leg nodes (ThC, CTr)")
    ax.set_ylabel("Frame count")
    ax.set_title(f"Pooled distribution ({len(all_values)} frames, {n_trials} trials)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    logger.info(f"Saved {output_path}")


def plot_per_trial_histograms(
    trial_names: list[str], trial_values: list[np.ndarray], output_path: Path
) -> None:
    n_cols = 5
    n_rows = -(-len(trial_names) // n_cols)
    fig, axes = plt.subplots(
        n_rows, n_cols, figsize=(3 * n_cols, 2 * n_rows), sharex=True
    )
    for ax, name, values in zip(axes.flat, trial_names, trial_values):
        ax.hist(values, bins=N_BINS, range=(0, 1), color="tab:blue")
        ax.axvline(FLIPPED_THRESHOLD, color="tab:red", linestyle="--", linewidth=1)
        ax.set_title(name, fontsize=7)
        ax.tick_params(labelsize=6)
    for ax in axes.flat[len(trial_names) :]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    logger.info(f"Saved {output_path}")


def save_samples(
    case_name: str,
    video_paths: list[Path],
    frame_idxs_per_trial: list[np.ndarray],
    mask_per_trial: list[np.ndarray],
    rng: np.random.Generator,
) -> None:
    """Randomly samples up to `N_SAMPLES_PER_CASE` frames (pooled across
    trials) where `mask_per_trial[trial]` is True, reads them from that
    trial's own aligned-domain video, resizes to `SAMPLE_SIZE`, and saves
    them as JPEGs under `OUTPUT_DIR / f"{case_name}_samples"`.
    """
    pooled = [
        (trial_idx, frame_idx)
        for trial_idx, (frame_idxs, mask) in enumerate(
            zip(frame_idxs_per_trial, mask_per_trial)
        )
        for frame_idx in frame_idxs[mask]
    ]

    n_samples = min(N_SAMPLES_PER_CASE, len(pooled))
    if n_samples < N_SAMPLES_PER_CASE:
        logger.warning(
            f"{case_name}: only {len(pooled)} candidate frame(s), sampling all of them"
        )
    chosen_idxs = rng.choice(len(pooled), size=n_samples, replace=False)

    by_trial: dict[int, list[int]] = {}
    for pooled_idx in chosen_idxs:
        trial_idx, frame_idx = pooled[pooled_idx]
        by_trial.setdefault(trial_idx, []).append(frame_idx)

    output_dir = OUTPUT_DIR / f"{case_name}_samples"
    output_dir.mkdir(parents=True, exist_ok=True)
    jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    n_saved = 0
    with tqdm(total=n_samples, desc=case_name, mininterval=1.0) as pbar:
        for trial_idx, frame_idxs in by_trial.items():
            genotype, fly_trial = parse_trial_identity(video_paths[trial_idx])
            sorted_idxs = sorted(frame_idxs)
            frames, _fps = pvio.read_frames_from_video(
                video_paths[trial_idx], sorted_idxs
            )
            for frame_idx, frame in zip(sorted_idxs, frames):
                resized = cv2.resize(frame, SAMPLE_SIZE, interpolation=cv2.INTER_AREA)
                resized_bgr = cv2.cvtColor(
                    resized, cv2.COLOR_RGB2BGR
                )  # pvio reads RGB.
                filename = f"{genotype}__{fly_trial}__frame{frame_idx:09d}.jpg"
                cv2.imwrite(str(output_dir / filename), resized_bgr, jpeg_params)
                n_saved += 1
                pbar.update(1)
            pbar.set_postfix_str(f"{genotype}__{fly_trial}")
    logger.info(f"{case_name}: saved {n_saved} image(s) -> {output_dir}")


def main() -> None:
    h5_paths = sorted(DATA_DIR.glob("*_pose2d_final_predictions.h5"))
    if not h5_paths:
        raise SystemExit(
            f"No *_pose2d_final_predictions.h5 files found under {DATA_DIR}"
        )
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    trial_names, trial_values, trial_frame_idxs, trial_video_paths = [], [], [], []
    for h5_path in h5_paths:
        stem = h5_path.name.removesuffix("_pose2d_final_predictions.h5")
        frame_idxs, confidence, video_path, n_hand = load_trial_confidence(h5_path)
        trial_names.append(stem)
        trial_values.append(confidence)
        trial_frame_idxs.append(frame_idxs)
        trial_video_paths.append(video_path)
        logger.info(
            f"{stem}: {len(confidence)} frames ({n_hand} hand-labeled excluded)"
        )

    all_values = np.concatenate(trial_values)
    logger.info(f"Pooled {len(all_values)} frames across {len(h5_paths)} trials")

    np.savez(
        OUTPUT_DIR / "flip_confidence.npz",
        trial_names=np.array(trial_names),
        **{f"{name}__confidence": v for name, v in zip(trial_names, trial_values)},
        **{
            f"{name}__frame_idxs": idxs
            for name, idxs in zip(trial_names, trial_frame_idxs)
        },
    )

    plot_pooled_histogram(
        all_values, len(h5_paths), OUTPUT_DIR / "flip_confidence_pooled_hist.png"
    )
    plot_per_trial_histograms(
        trial_names, trial_values, OUTPUT_DIR / "flip_confidence_per_trial_hist.png"
    )

    flipped_masks = [is_flipped(values) for values in trial_values]
    unflipped_masks = [~mask for mask in flipped_masks]
    logger.info(
        f"threshold={FLIPPED_THRESHOLD}: "
        f"{sum(m.sum() for m in flipped_masks)} flipped, "
        f"{sum(m.sum() for m in unflipped_masks)} unflipped candidates"
    )

    rng = np.random.default_rng(SEED)
    save_samples("flipped", trial_video_paths, trial_frame_idxs, flipped_masks, rng)
    save_samples("unflipped", trial_video_paths, trial_frame_idxs, unflipped_masks, rng)


if __name__ == "__main__":
    main()
