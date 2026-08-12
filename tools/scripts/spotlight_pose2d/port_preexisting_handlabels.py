#!/usr/bin/env python
"""Port the hand-corrected labels in `labels/first_round/labels.v005.slp`
(legacy format, raw fullsize domain, 4 videos) into the matching trials'
ported LM dataset (`labels/ported_lm_predictions/*_slp16_aligned_promoted.slp`).

`labels.v005.slp`'s first video (an old, unrelated experiment) has no
matching trial under `june2026/` and is skipped automatically; its other
three videos are matched to a trial by parsing their
`.../june2026/<genotype>/<fly_trial>[_full]/processed/fullsize_behavior_video*.mkv`
path (ignoring the `_full` trial-dir suffix and whatever follows
`fullsize_behavior_video` in the filename itself).

For each matched trial, only the frames with a real (non-predicted)
instance in `labels.v005.slp` are ported: transformed from the raw
fullsize domain into the aligned domain (via `slp_apply_alignment.py`,
using that trial's own transforms on the NAS), then spliced into a copy of
that trial's `..._slp16_aligned_promoted.slp`, replacing whatever
prediction or confidence-promoted label was already at that frame. Output
is a new `..._slp16_aligned_promoted_handlabeled.slp` per trial; the
existing `_slp16_aligned_promoted.slp` files (and `merged_promoted.slp`)
are left untouched.

Hardcoded constants, no CLI args.

Usage:
    python scripts/spotlight_pose2d/port_preexisting_handlabels.py
"""

import subprocess
from pathlib import Path

import sleap_io as sio
from loguru import logger

from spotlight_tools.spotlight_pose2d.io_utils import (
    parse_genotype_trial,
    rebuild_instance,
    user_labeled_frames,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_DIR = REPO_ROOT / "tools/spotlight_pose2d"
FIRST_ROUND_SLP = (
    REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/first_round/labels.v005.slp"
)
DATA_ROOT = (
    REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"
)
NAS_ROOT = Path("/mnt/upramdya_data/VAS/poseforge_paper_data")
FULLSIZE_VIDEO_RELPATH = Path("processed/fullsize_behavior_video.mkv")


def port_trial(genotype: str, fly_trial: str, frames: list[tuple[int, list]]) -> int:
    """Port one trial's hand-labeled frames; returns the number ported."""
    stem = f"{genotype}__{fly_trial}"
    promoted_path = DATA_ROOT / f"{stem}_slp16_aligned_promoted.slp"
    output_path = DATA_ROOT / f"{stem}_slp16_aligned_promoted_handlabeled.slp"
    if not promoted_path.is_file():
        logger.warning(f"{stem}: no {promoted_path.name}, skipping")
        return 0
    if output_path.is_file():
        logger.info(f"{stem}: {output_path.name} already exists, skipping")
        return 0

    fullsize_video = sio.Video(filename=str(FULLSIZE_VIDEO_RELPATH))
    fullsize_labels = sio.Labels(
        labeled_frames=[
            sio.LabeledFrame(
                video=fullsize_video, frame_idx=frame_idx, instances=instances
            )
            for frame_idx, instances in frames
        ],
        videos=[fullsize_video],
        skeletons=[frames[0][1][0].skeleton],
    )
    fullsize_path = DATA_ROOT / f"{stem}_handlabels_fullsize.slp"
    sio.save_file(fullsize_labels, str(fullsize_path))

    aligned_path = DATA_ROOT / f"{stem}_handlabels_aligned.slp"
    align_cmd = [
        "uv", "run", "--project", str(REPO_ROOT), "python",
        str(TOOLS_DIR / "slp_apply_alignment.py"), "--align",
        "--input-path", str(fullsize_path),
        "--output-path", str(aligned_path),
        "--override",
    ]  # fmt: skip
    subprocess.run(align_cmd, cwd=NAS_ROOT / genotype / fly_trial, check=True)
    aligned_labels = sio.load_file(str(aligned_path))
    aligned_by_frame_idx = {
        lf.frame_idx: lf.instances for lf in aligned_labels.labeled_frames
    }

    promoted = sio.load_file(str(promoted_path))
    new_labeled_frames = []
    n_ported = 0
    for lf in promoted.labeled_frames:
        instances = aligned_by_frame_idx.get(lf.frame_idx)
        if instances is None:
            new_labeled_frames.append(lf)
            continue
        rebuilt = [rebuild_instance(inst, promoted.skeleton) for inst in instances]
        new_labeled_frames.append(
            sio.LabeledFrame(video=lf.video, frame_idx=lf.frame_idx, instances=rebuilt)
        )
        n_ported += 1

    promoted.labeled_frames = new_labeled_frames
    sio.save_file(promoted, str(output_path))
    logger.info(
        f"{stem}: ported {n_ported}/{len(frames)} hand-labeled frames -> {output_path}"
    )
    return n_ported


def main() -> None:
    labels = sio.load_file(str(FIRST_ROUND_SLP))

    total_ported = 0
    for video in labels.videos:
        parsed = parse_genotype_trial(video.filename)
        if parsed is None:
            logger.info(f"Skipping unmatched video: {video.filename}")
            continue
        genotype, fly_trial = parsed

        frames = user_labeled_frames(labels, video)
        if not frames:
            logger.warning(
                f"{genotype}__{fly_trial}: no hand-labeled frames found, skipping"
            )
            continue

        total_ported += port_trial(genotype, fly_trial, frames)

    logger.info(f"Done: ported {total_ported} hand-labeled frames in total")


if __name__ == "__main__":
    main()
