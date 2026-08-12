#!/usr/bin/env python
"""Promote confident, non-redundant predicted instances to user labels.

Takes a `.slp` file containing `PredictedInstance` data (e.g. the output of
`slp_run_inference.py`, aligned via `slp_apply_alignment.py` if needed) and,
for every frame that does not already have a user-labeled `Instance`,
decides whether to add one (linked to the predicted instance via
`from_predicted`) based on:

- Per-keypoint confidence: detection score and same-leg segment length (see
  `compute_promotion_mask`).
- Non-redundancy: a frame is skipped if its confident keypoints are
  essentially unchanged from the most recently kept frame, whether that
  frame was already labeled or was itself just promoted (see
  `compute_novel_pose_mask`).

Frames that already have a user-labeled `Instance` are left untouched, but
still anchor the non-redundancy check so newly-promoted frames aren't
near-duplicates of them either.

`--aligned-input` (default False) documents whether the input's coordinates
are in the aligned domain, which `max_leg_segment_length` and
`min_pose_change` both assume (an affine alignment transform is a rigid
rotation+translation with no scaling, so pixel distances happen to be
unaffected by it, but this isn't guaranteed for other kinds of
"already-aligned" input); a warning is logged if False.

Requires current-format input (see `slp_convert_legacy.py`): unlike
`sleap_io`'s own transparent read support for the legacy format, this tool
doesn't accept it, since round-tripping a legacy file's own quirks through
promotion hasn't been verified.

Usage:
    python slp_promote_by_confidence.py \\
        --input-path predictions.slp --output-path promoted.slp
"""

from itertools import pairwise
from pathlib import Path

import numpy as np
import sleap_io as sio
import tyro
from loguru import logger
from slp_convert_legacy import is_legacy_format

from spotlight_tools.spotlight_pose2d.io_utils import check_output_path

# Each leg's chain of adjacent joints, in order. Node names are "{prefix}_{joint}".
LEG_PREFIXES = ("LF", "LM", "LH", "RF", "RM", "RH")
LEG_JOINTS = ("ThC", "CTr", "FTi", "TiTa", "Cl")


def compute_promotion_mask(
    points: np.ndarray,
    keypoint_scores: np.ndarray,
    node_names: list[str],
    min_keypoint_score: float,
    max_leg_segment_length: float,
    max_missing_keypoints: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Decide which frames and keypoints are confident enough to promote to labels.

    A keypoint is "confident" if it was detected (non-NaN), its detection
    score is at least `min_keypoint_score`, and it does not stretch any
    adjacent same-leg segment (`ThC-CTr`, `CTr-FTi`, `FTi-TiTa`, `TiTa-Cl`)
    beyond `max_leg_segment_length` pixels: a real leg segment has a fixed
    length, so a large distance between two adjacent joints means at least
    one of them was mislocalized. Both endpoints of an over-long segment are
    marked not confident, since which one is wrong isn't known. This check
    does not apply to any other pair of nodes.

    A frame is promoted, meaning its predicted instance becomes a user-labeled
    instance, if at most `max_missing_keypoints` of its keypoints are not
    confident.

    Args:
        points: `(n_frames, n_nodes, 2)` keypoint coordinates, NaN where undetected.
        keypoint_scores: `(n_frames, n_nodes)` per-keypoint detection scores.
        node_names: Node name per column of `points`/`keypoint_scores`.
        min_keypoint_score: Minimum per-keypoint score to be considered confident.
        max_leg_segment_length: Maximum pixel distance between adjacent
            same-leg joints.
        max_missing_keypoints: Maximum number of not-confident keypoints a
            frame's predicted instance may have and still be promoted.

    Returns:
        keypoint_visible: `(n_frames, n_nodes)` bool, keypoints confident enough
            to include in a promoted label.
        frame_promoted: `(n_frames,)` bool, frames whose predicted instance is
            promoted to a user label.
    """
    n_nodes = points.shape[1]
    present = ~np.isnan(points).any(axis=-1)
    score_ok = keypoint_scores >= min_keypoint_score
    keypoint_visible = present & score_ok

    name_to_idx = {name: i for i, name in enumerate(node_names)}
    for leg in LEG_PREFIXES:
        joint_indices = [name_to_idx[f"{leg}_{joint}"] for joint in LEG_JOINTS]
        for a, b in pairwise(joint_indices):
            segment_length = np.linalg.norm(points[:, a] - points[:, b], axis=-1)
            too_long = segment_length > max_leg_segment_length  # False where NaN
            keypoint_visible[too_long, a] = False
            keypoint_visible[too_long, b] = False

    n_missing = n_nodes - keypoint_visible.sum(axis=1)
    frame_promoted = n_missing <= max_missing_keypoints
    return keypoint_visible, frame_promoted


def compute_novel_pose_mask(
    points: np.ndarray,
    keypoint_visible: np.ndarray,
    candidate: np.ndarray,
    min_pose_change: float,
) -> np.ndarray:
    """Among candidate frames, mark those that differ from the last kept one.

    A resting or asleep fly can produce long runs of frames that are all
    essentially identical; promoting every one of them to a label adds no
    information and just floods the label set with redundant repeats. This
    walks through the candidate frames in order and only keeps one once its
    confident keypoints have moved, on average, more than `min_pose_change`
    pixels from the most recently kept candidate (comparing only nodes
    confident in both). This naturally thins out both exact freezes and slow
    drift alike, since a gradually drifting pose still eventually accumulates
    enough displacement to count as novel. Non-candidate frames are ignored:
    they neither count as novel nor update the reference, so a low-confidence
    frame can't corrupt what later frames are compared against.

    Args:
        points: `(n_frames, n_nodes, 2)` keypoint coordinates (same domain as
            what will be saved, i.e. aligned).
        keypoint_visible: `(n_frames, n_nodes)` bool, confident keypoints from
            `compute_promotion_mask` (or, for already-labeled frames, that
            label's own visibility).
        candidate: `(n_frames,)` bool, frames to walk through in order: either
            already labeled, or eligible for promotion.
        min_pose_change: Minimum mean per-keypoint displacement, in pixels,
            from the last kept frame for a frame to count as novel.

    Returns:
        `(n_frames,)` bool, True for candidate frames that are not
        near-duplicates of the most recently kept candidate frame.
    """
    n_frames = points.shape[0]
    is_novel = np.zeros(n_frames, dtype=bool)
    last_points = None
    last_visible = None
    for i in range(n_frames):
        if not candidate[i]:
            continue
        visible = keypoint_visible[i]
        if last_points is None:
            novel = True
        else:
            shared = visible & last_visible
            if not shared.any():
                novel = True
            else:
                displacement = np.linalg.norm(
                    points[i, shared] - last_points[shared], axis=-1
                )
                novel = bool(displacement.mean() > min_pose_change)
        is_novel[i] = novel
        if novel:
            last_points = points[i]
            last_visible = visible
    return is_novel


def extract_video_arrays(
    labels: sio.Labels, video: sio.Video, node_names: list[str]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Build dense per-frame arrays for one video's labeled frames.

    Args:
        labels: The full `Labels` object (frames from other videos are ignored).
        video: The video to extract frames for.
        node_names: Node name per column, in skeleton order.

    Returns:
        poses: `(n_frames, n_nodes, 2)` coordinates: the existing user
            instance's points where already labeled, else the best predicted
            instance's.
        keypoint_scores: `(n_frames, n_nodes)` per-keypoint detection scores.
        already_labeled: `(n_frames,)` bool, True where a user `Instance`
            already exists.
        frame_lookup: `{frame_idx: (LabeledFrame, PredictedInstance)}` for
            frames with a predicted instance, for writing results back.
    """
    n_nodes = len(node_names)
    frames_for_video = [lf for lf in labels.labeled_frames if lf.video is video]
    n_frames = max((lf.frame_idx for lf in frames_for_video), default=-1) + 1

    poses = np.full((n_frames, n_nodes, 2), np.nan, dtype=np.float32)
    keypoint_scores = np.full((n_frames, n_nodes), np.nan, dtype=np.float32)
    already_labeled = np.zeros(n_frames, dtype=bool)
    frame_lookup = {}

    for lf in frames_for_video:
        # PredictedInstance is a subclass of Instance, so exclude it explicitly
        # to isolate actual user-labeled instances.
        user_instances = [
            inst for inst in lf.instances if not isinstance(inst, sio.PredictedInstance)
        ]
        predicted = [
            inst for inst in lf.instances if isinstance(inst, sio.PredictedInstance)
        ]

        if predicted:
            best = max(predicted, key=lambda inst: inst.score)
            pts_scores = best.numpy(scores=True)  # (n_nodes, 3): x, y, score
            poses[lf.frame_idx] = pts_scores[:, :2]
            keypoint_scores[lf.frame_idx] = pts_scores[:, 2]
            frame_lookup[lf.frame_idx] = (lf, best)

        if user_instances:
            already_labeled[lf.frame_idx] = True
            # An already-labeled frame anchors the non-redundancy check using
            # its own (trusted) points rather than the raw prediction.
            poses[lf.frame_idx] = user_instances[0].numpy()

    return poses, keypoint_scores, already_labeled, frame_lookup


def filter_video(
    labels: sio.Labels,
    video: sio.Video,
    node_names: list[str],
    min_keypoint_score: float,
    max_leg_segment_length: float,
    max_missing_keypoints: int,
    min_pose_change: float,
) -> None:
    """Promote confident, non-redundant frames of one video to user labels, in place."""
    poses, keypoint_scores, already_labeled, frame_lookup = extract_video_arrays(
        labels, video, node_names
    )

    keypoint_visible, meets_criteria = compute_promotion_mask(
        poses,
        keypoint_scores,
        node_names,
        min_keypoint_score,
        max_leg_segment_length,
        max_missing_keypoints,
    )
    # Already-labeled frames anchor the non-redundancy walk but are never
    # reconsidered for promotion themselves.
    eligible = meets_criteria & ~already_labeled
    is_novel = compute_novel_pose_mask(
        poses, keypoint_visible, already_labeled | eligible, min_pose_change
    )
    to_promote = eligible & is_novel

    n_promoted = 0
    for frame_idx in np.flatnonzero(to_promote):
        lf, predicted_instance = frame_lookup[int(frame_idx)]
        label_points = poses[frame_idx].copy()
        label_points[~keypoint_visible[frame_idx]] = np.nan
        lf.instances.append(
            sio.Instance.from_numpy(
                points_data=label_points,
                skeleton=labels.skeleton,
                from_predicted=predicted_instance,
            )
        )
        n_promoted += 1

    logger.info(
        f"{video.filename}: {len(frame_lookup)} predicted frames, "
        f"{already_labeled.sum()} already labeled, {n_promoted} newly promoted"
    )


def main(
    input_path: Path,
    output_path: Path,
    aligned_input: bool = False,
    min_keypoint_score: float = 0.5,
    max_leg_segment_length: float = 200.0,
    max_missing_keypoints: int = 0,
    min_pose_change: float = 2.0,
    override: bool = False,
) -> None:
    """Promote confident, non-redundant predicted instances to user labels.

    Args:
        input_path: `.slp` file with `PredictedInstance` data to filter.
        output_path: Where to save the filtered `.slp` file. Aborts if this
            already exists, unless `override` is set.
        aligned_input: Whether `input_path`'s coordinates are already in the
            aligned domain, which the pixel-based thresholds below assume.
            Purely informational (a warning is logged if False); this tool
            does not itself transform coordinates.
        min_keypoint_score: Minimum per-keypoint detection score to promote a
            keypoint to a label.
        max_leg_segment_length: Maximum pixel distance between adjacent
            same-leg joints. See `compute_promotion_mask`.
        max_missing_keypoints: Maximum number of not-confident keypoints a
            frame may have and still be promoted. See `compute_promotion_mask`.
        min_pose_change: Minimum mean per-keypoint displacement, in pixels,
            for a frame to count as a new pose rather than a near-duplicate of
            the last kept label. See `compute_novel_pose_mask`.
        override: If True, overwrite `output_path` if it already exists.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    if is_legacy_format(input_path):
        raise SystemExit(
            f"{input_path} is in the legacy SLEAP format; convert it first "
            "with `slp_convert_legacy.py --legacy-to-current`."
        )
    check_output_path(output_path, override)

    if not aligned_input:
        logger.warning(
            "--aligned-input is False: max_leg_segment_length and min_pose_change "
            "assume aligned-domain coordinates; results may not be meaningful otherwise."
        )

    labels = sio.load_file(str(input_path))
    node_names = [node.name for node in labels.skeleton.nodes]

    for video in labels.videos:
        filter_video(
            labels,
            video,
            node_names,
            min_keypoint_score,
            max_leg_segment_length,
            max_missing_keypoints,
            min_pose_change,
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    sio.save_file(labels, str(output_path))
    logger.info(f"Saved filtered labels to {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
