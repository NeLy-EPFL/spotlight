#!/usr/bin/env python
"""Transform predictions/labels between the raw camera domain and the
aligned (cropped) video domain.

`--align` maps points from each video's raw fullsize domain (this tool's
input, linked from `processed/fullsize_behavior_video.mkv`) into its own
aligned video's domain (output linked from
`processed/aligned_behavior_video.mkv`); `--unalign` does the inverse. Each
video's per-frame affine transform comes from that video's own linked
trial directory (`processed/behavior_alignment_transforms.h5`), so no
separate `--data-root` is needed; a video whose trial has no transform (or
no target video) is skipped with a warning, not an error, so a
multi-trial-in-one-file input can still align the trials that are ready.
Every instance in a frame (both predictions and any existing labels) is
transformed, preserving its kind and scores.

Requires current-format input (see `slp_convert_legacy.py`): unlike
`sleap_io`'s own transparent read support for the legacy format, this tool
doesn't accept it, since round-tripping a legacy file's own quirks through a
geometric transform hasn't been verified.

Usage:
    python slp_apply_alignment.py --align \\
        --input-path raw_predictions.slp --output-path aligned_predictions.slp
"""

from pathlib import Path

import h5py
import numpy as np
import sleap_io as sio
import tyro
from loguru import logger
from slp_convert_legacy import is_legacy_format

from spotlight_postprocessing.pose2d.geometry import (
    apply_affine,
    invert_affine,
)
from spotlight_postprocessing.pose2d.io_utils import (
    check_output_path,
    trial_dir_from_video_path,
    try_resolve_video_path,
)

FULLSIZE_VIDEO_RELPATH = Path("processed/fullsize_behavior_video.mkv")
ALIGNED_VIDEO_RELPATH = Path("processed/aligned_behavior_video.mkv")
TRANSFORMS_RELPATH = Path("processed/behavior_alignment_transforms.h5")


def load_transform_matrices(trial_dir: Path) -> np.ndarray:
    """Load a trial's per-frame raw-to-aligned affine transforms.

    Args:
        trial_dir: Trial directory containing `TRANSFORMS_RELPATH`.

    Returns:
        `(n_frames, 2, 3)` affine matrices, one per frame.
    """
    with h5py.File(trial_dir / TRANSFORMS_RELPATH, "r") as f:
        return f["transform_matrices"][:]


def transform_instance(
    instance: sio.Instance, matrix: np.ndarray, skeleton: sio.Skeleton
) -> sio.Instance:
    """Apply one frame's affine `matrix` to one instance, preserving its kind."""
    points = instance.numpy()[np.newaxis]  # (1, n_nodes, 2)
    new_points = apply_affine(points, matrix[np.newaxis])[0]
    if isinstance(instance, sio.PredictedInstance):
        return sio.PredictedInstance.from_numpy(
            points_data=new_points,
            skeleton=skeleton,
            point_scores=instance.numpy(scores=True)[:, 2],
            score=instance.score,
        )
    return sio.Instance.from_numpy(points_data=new_points, skeleton=skeleton)


def transform_labeled_frames(
    labels: sio.Labels,
    video: sio.Video,
    new_video: sio.Video,
    transform_matrices: np.ndarray,
    align: bool,
) -> list[sio.LabeledFrame]:
    """Transform every instance of every labeled frame for `video`.

    Args:
        labels: The full `Labels` object (frames from other videos are ignored).
        video: The (single) video `labels`' frames are linked to.
        new_video: The video the transformed frames should link to instead.
        transform_matrices: `(n_frames, 2, 3)` raw-to-aligned affine
            matrices, covering the whole video (see `load_transform_matrices`).
        align: If True, apply the matrices as-is (raw -> aligned); if False,
            apply their inverse (aligned -> raw).

    Returns:
        New `LabeledFrame`s, linked to `new_video`.
    """
    n_frames = transform_matrices.shape[0]
    new_labeled_frames = []
    for lf in labels.labeled_frames:
        if lf.video is not video:
            continue
        if lf.frame_idx >= n_frames:
            raise SystemExit(
                f"Frame {lf.frame_idx} is out of range for the {n_frames}-frame "
                "transform (mismatched video/trial?)."
            )
        matrix = transform_matrices[lf.frame_idx]
        if not align:
            matrix = invert_affine(matrix[np.newaxis])[0]
        new_instances = [
            transform_instance(inst, matrix, labels.skeleton) for inst in lf.instances
        ]
        new_labeled_frames.append(
            sio.LabeledFrame(
                video=new_video, frame_idx=lf.frame_idx, instances=new_instances
            )
        )
    return new_labeled_frames


def main(
    input_path: Path,
    output_path: Path,
    align: bool = False,
    unalign: bool = False,
    override: bool = False,
) -> None:
    """Transform predictions/labels to/from the aligned video domain.

    Args:
        input_path: Current-format `.slp` file to transform (one or more videos).
        output_path: Where to save the transformed `.slp` file. Aborts if
            this already exists, unless `override` is set.
        align: Map from each video's raw fullsize domain to its aligned
            domain. Exactly one of `align`/`unalign` must be set.
        unalign: Map from each video's aligned domain to its raw fullsize
            domain. Exactly one of `align`/`unalign` must be set.
        override: If True, overwrite `output_path` if it already exists.
    """
    if align == unalign:
        raise SystemExit("Specify exactly one of --align or --unalign.")
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    if is_legacy_format(input_path):
        raise SystemExit(
            f"{input_path} is in the legacy SLEAP format; convert it first "
            "with `slp_convert_legacy.py --legacy-to-current`."
        )
    check_output_path(output_path, override)

    labels = sio.load_file(str(input_path))
    new_videos = []
    all_new_labeled_frames = []
    n_skipped = 0
    for video in labels.videos:
        video_path = try_resolve_video_path(video.filename)
        if video_path is None:
            logger.warning(
                f"Skipping {video.filename}: does not resolve to an existing file"
            )
            n_skipped += 1
            continue
        trial_dir = trial_dir_from_video_path(video_path)

        transforms_path = trial_dir / TRANSFORMS_RELPATH
        new_video_path = trial_dir / (
            ALIGNED_VIDEO_RELPATH if align else FULLSIZE_VIDEO_RELPATH
        )
        if not transforms_path.is_file() or not new_video_path.is_file():
            logger.warning(
                f"Skipping {video.filename}: "
                f"{'missing transforms at ' + str(transforms_path) if not transforms_path.is_file() else ''}"
                f"{'missing video at ' + str(new_video_path) if not new_video_path.is_file() else ''}"
            )
            n_skipped += 1
            continue

        transform_matrices = load_transform_matrices(trial_dir)
        new_video = sio.Video(filename=str(new_video_path))
        new_videos.append(new_video)
        all_new_labeled_frames.extend(
            transform_labeled_frames(
                labels, video, new_video, transform_matrices, align
            )
        )

    if not new_videos:
        raise SystemExit("No video could be transformed (see warnings above).")

    new_labels = sio.Labels(
        labeled_frames=all_new_labeled_frames,
        videos=new_videos,
        skeletons=[labels.skeleton],
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    sio.save_file(new_labels, str(output_path))
    direction = "Aligned" if align else "Unaligned"
    logger.info(
        f"{direction} {len(all_new_labeled_frames)} frames across {len(new_videos)} "
        f"video(s) ({n_skipped} skipped) to {output_path}"
    )


if __name__ == "__main__":
    tyro.cli(main)
