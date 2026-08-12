#!/usr/bin/env python
"""Runs a pose2d checkpoint over every trial's candidate frames under
`data_root` (globbing `*_pose.h5`), producing a fresh (unpromoted)
prediction per candidate frame and a per-trial `.slp`, then merges all of
them into one multi-video `.slp`. See `predict_unlabeled_frames.py`'s own
docstring for what "candidate" means and why this project's confidence-
based auto-promotion isn't trusted as a real label here.

`--frames-per-trial`, if set, only predicts that many randomly-sampled
(fixed seed 0, independently per trial) candidate frames per trial instead
of every one -- for a fast per-round quality check, meant to be re-run
after every training pass. Leave it unset for exhaustive, every-frame
predictions (e.g. for a real labeling round rather than a spot check).

Genuine hand-labeled frames (protected from re-prediction) are derived
from `--first-round-slp-path`, by matching its videos' paths to trials
under `data_root` via `io_utils.parse_genotype_trial`'s own matching
logic, unioned with any genuinely corrected (non-predicted) instances in
`--corrected-slp-paths` -- e.g. a prior round's own merged sample
predictions, after GUI review -- so a later round's checkpoint doesn't
silently overwrite corrections a human already made. `data_root`'s own
`.h5` files never see those corrections (they aren't retroactively
updated), so this is the only thing that protects them here.

Sequential, not parallel: unlike `cache_video_frames.py`-based caching
(CPU-bound), this is GPU-bound, so running several trials at once on the
same GPU would just make them contend with each other rather than speed
anything up.

Usage:
    python tools/spotlight_pose2d/sample_predictions_after_training.py \\
        --checkpoint-path bulk_data/.../checkpoints/iter1a/best.pt \\
        --data-root bulk_data/.../labels/ported_lm_predictions \\
        --output-dir bulk_data/.../labels/rounds/iter1a \\
        --frame-cache-root bulk_data/motion_prior/2dpose_model/frame_cache \\
        --first-round-slp-path bulk_data/.../labels/first_round/labels.v005.slp \\
        --corrected-slp-paths bulk_data/.../rounds/iter0c/merged_pose2d_iter0c_sample_predictions_corrected.slp \\
        --skeleton-json-path bulk_data/.../labels/metadata.json \\
        --output-suffix _pose2d_iter1a_sample_predictions \\
        --merged-slp-path bulk_data/.../merged_pose2d_iter1a_sample_predictions.slp \\
        --frames-per-trial 2000
"""

from pathlib import Path

import sleap_io as sio
import tyro
from loguru import logger
from predict_unlabeled_frames import main as predict_unlabeled
from slp_merge import main as merge_slp

from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    parse_genotype_trial,
    parse_trial_identity,
    user_labeled_frames,
)


def true_hand_label_indices_by_trial(
    first_round_slp_path: Path,
) -> dict[str, list[int]]:
    """`{"<genotype>__<fly_trial>": [frame_idx, ...]}` for every trial with
    a genuine hand label in `first_round_slp_path`, per
    `io_utils.parse_genotype_trial`'s own matching logic (empty for any
    trial it doesn't touch).
    """
    labels = sio.load_file(str(first_round_slp_path))
    by_trial: dict[str, list[int]] = {}
    for video in labels.videos:
        parsed = parse_genotype_trial(video.filename)
        if parsed is None:
            continue
        genotype, fly_trial = parsed
        frames = user_labeled_frames(labels, video)
        by_trial[f"{genotype}__{fly_trial}"] = sorted(idx for idx, _ in frames)
    return by_trial


def corrected_frame_indices_by_trial(corrected_slp_path: Path) -> dict[str, list[int]]:
    """`{"<genotype>__<fly_trial>": [frame_idx, ...]}` for every trial with
    a genuinely corrected (non-predicted) instance in `corrected_slp_path`
    -- an aligned-domain merged `.slp` (e.g. a prior round's own sample
    predictions, after GUI review), keyed by `parse_trial_identity` rather
    than `io_utils.parse_genotype_trial`, since
    this is a different video-path convention (aligned, not raw fullsize).
    """
    labels = sio.load_file(str(corrected_slp_path))
    by_trial: dict[str, list[int]] = {}
    for video in labels.videos:
        genotype, fly_trial = parse_trial_identity(video.filename)
        frames = [
            lf.frame_idx
            for lf in labels.labeled_frames
            if lf.video is video
            and any(
                not isinstance(inst, sio.PredictedInstance) for inst in lf.instances
            )
        ]
        if frames:
            by_trial[f"{genotype}__{fly_trial}"] = sorted(frames)
    return by_trial


def main(
    checkpoint_path: Path,
    data_root: Path,
    frame_cache_root: Path,
    first_round_slp_path: Path,
    skeleton_json_path: Path,
    output_suffix: str,
    merged_slp_path: Path,
    output_dir: Path | None = None,
    corrected_slp_paths: list[Path] | None = None,
    frames_per_trial: int | None = None,
    n_keypoints: int = 37,
    batch_size: int = 200,
) -> None:
    """Predict every trial's dense pose `.h5` under `data_root`.

    Args:
        checkpoint_path: Trained `RepVGGPoseModel` state dict.
        data_root: Directory containing each trial's `*_pose.h5` (see
            `spotlight_postprocessing.spotlight_pose2d.io_utils.save_pose_h5`);
            every match gets predicted. Shared across rounds, so this stays
            flat rather than moving into a per-round subfolder.
        frame_cache_root: Root directory `cache_video_frames.py` wrote
            into; must already have every trial cached at `INPUT_SIZE`.
        first_round_slp_path: `.slp` file with genuine hand labels (see
            `port_preexisting_handlabels.py`, the tool that originally
            substituted them in) to protect from re-prediction.
        skeleton_json_path: JSON file with the real skeleton (nodes,
            edges, symmetries), from `extract_metadata_from_initial_slp.py`,
            passed through to `predict_unlabeled_frames.py` for every trial.
        output_suffix: Appended to each trial's stem for its own output
            `.h5`/`.slp` (e.g. `_pose2d_iter0c_sample_predictions`).
        merged_slp_path: Where to save all trials' `.slp` files merged
            into one multi-video `.slp` (via `slp_merge.py`). Aborts if
            this already exists.
        corrected_slp_paths: Aligned-domain merged `.slp` file(s) (e.g. a
            prior round's own sample predictions, after GUI review) whose
            genuinely corrected frames should also be protected from
            re-prediction, on top of `first_round_slp_path`'s. `data_root`'s
            own `.h5` files are never updated with these corrections, so
            without this a later round would silently overwrite them.
        output_dir: Directory to write each trial's output `.h5`/`.slp`
            into, e.g. a `data_root/<round>/` subfolder so a round's
            outputs don't mix with `data_root`'s shared input files or
            another round's outputs. Defaults to `data_root` itself.
        frames_per_trial: If set, only predict this many randomly-sampled
            (fixed seed 0, independent per trial) candidate frames per
            trial, instead of every one -- for a fast per-round quality
            check. Unset predicts every candidate frame.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        batch_size: Frames per inference batch.
    """
    output_dir = output_dir or data_root
    output_dir.mkdir(parents=True, exist_ok=True)
    protected_by_trial = true_hand_label_indices_by_trial(first_round_slp_path)
    for corrected_slp_path in corrected_slp_paths or []:
        for stem, frames in corrected_frame_indices_by_trial(
            corrected_slp_path
        ).items():
            protected_by_trial[stem] = sorted(
                set(protected_by_trial.get(stem, [])) | set(frames)
            )

    h5_paths = sorted(data_root.glob("*_pose.h5"))
    slp_paths = []
    for trial_idx, h5_path in enumerate(h5_paths, start=1):
        stem = h5_path.name.removesuffix("_pose.h5")
        output_h5_path = output_dir / f"{stem}{output_suffix}.h5"
        output_slp_path = output_dir / f"{stem}{output_suffix}.slp"
        slp_paths.append(output_slp_path)
        if output_h5_path.is_file():
            logger.info(
                f"[{trial_idx}/{len(h5_paths)}] Skipping {stem} (already predicted)"
            )
            continue

        protected = protected_by_trial.get(stem, [])
        logger.info(
            f"=== [{trial_idx}/{len(h5_paths)}] {stem} "
            f"({len(protected)} protected hand labels) ==="
        )
        predict_unlabeled(
            checkpoint_path=checkpoint_path,
            input_path=h5_path,
            frame_cache_root=frame_cache_root,
            skeleton_json_path=skeleton_json_path,
            output_h5_path=output_h5_path,
            output_slp_path=output_slp_path,
            protected_frame_indices=protected,
            frames_per_trial=frames_per_trial,
            n_keypoints=n_keypoints,
            batch_size=batch_size,
        )

    logger.info(f"All {len(slp_paths)} trials predicted")

    if merged_slp_path.is_file():
        logger.info(f"Skipping merge ({merged_slp_path} already exists)")
    else:
        logger.info(f"Merging {len(slp_paths)} trial .slp files -> {merged_slp_path}")
        merge_slp(input_paths=slp_paths, output_path=merged_slp_path)


if __name__ == "__main__":
    tyro.cli(main)
