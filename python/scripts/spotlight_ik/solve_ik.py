#!/usr/bin/env python
"""Convert one trial's dense pose2d `.h5` into `kinematics.h5`: physical
(mm) keypoint positions alongside the raw pixel predictions, and,
optionally, an inverse-kinematics fit (QuickIK) of the NeuroMechFly body
plan against those keypoints -- one row per frame of the whole trial, no
period/segment concept. Curating "which stretches are good enough for,
e.g., FlyGym snippet selection" now happens downstream in poseforge2,
using this file's own per-frame confidence/mismatch data, not a pre-baked
accept/reject decision made here.

IK is skipped (left NaN) for frames the localization model flagged flipped
(`flipped_prob >= FLIP_PROB_THRESHOLD`) or where the 2D pose's own
weighted confidence (`spotlight_localization.flip_label.weighted_confidence` --
the SAME proxy used to pseudo-label that model's own training data) is
below `WEIGHTED_CONFIDENCE_THRESHOLD`: both indicate the fly's own crop/
orientation is unreliable enough that fitting IK to it isn't meaningful,
independent of any downstream mismatch-based quality check. This is a
per-frame attempt/skip decision, not exposed as a period in the output --
IK still runs one contiguous good stretch at a time internally (QuickIK's
`SequenceSolver` warm-starts frame-to-frame, so a stretch's solve
shouldn't warm-start off an unrelated stretch across a skipped run -- see
`solve_period_ik`'s own docstring), but those stretch boundaries are
never written anywhere.

Note this is deliberately NOT the same thing as mismatch-based rejection
(how far the FK result ends up from the original 2D prediction): that's a
display-time decision for the QA video, not something that drops data
here (see `postprocessing.visualize`).

Fits the NeuroMechFly body plan (see `../flygym/scripts/
export_model_for_quickik.py`) via QuickIK's `SequenceSolver` with an
orthographic XY projection (see `spotlight_ik.neuromechfly.
solve_period_ik`). Only the 30 leg SLEAP nodes are used as real
observations; "Th" (thorax) never is (see `solve_period_ik`'s own
comment: it's the floating root, not a real keypoint on the body plan).

Usage:
    python scripts/spotlight_ik/solve_ik.py \\
        --input-path trial_pose2d_predictions.h5 \\
        --output-path trial_kinematics.h5
"""

import warnings
from pathlib import Path

import numpy as np
import tyro
from loguru import logger
from scipy import ndimage

from spotlight_postprocessing.spotlight_ik.calibration import (
    convert_mm_to_px,
    convert_px_to_mm,
    load_calibration_mapper,
    load_flipped_prob,
    load_stage_positions_mm,
    load_transform_matrices,
)
from spotlight_postprocessing.spotlight_ik.io_utils import (
    InverseKinematicsResult,
    save_kinematics_h5,
)
from spotlight_postprocessing.spotlight_ik.neuromechfly import (
    NEUTRAL_WEIGHT,
    BodyPlan,
    build_sleap_to_joint_name_map,
    sleap_keypoint_weight_scale,
    solve_period_ik,
)
from spotlight_postprocessing.spotlight_localization.flip_label import (
    weighted_confidence,
)
from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    trial_dir_from_video_path,
)
from spotlight_tools.common import get_assets_dir

DEFAULT_BODY_PLAN_PATH = get_assets_dir() / "neuromechfly_ypr_legs.json"

# Internal gap-detection thresholds (see module docstring) -- not exposed
# as CLI flags or written to the output file, since there's no "period"
# concept to configure anymore; only whether a frame gets a real IK
# result or NaN.
FLIP_PROB_THRESHOLD = 0.5
WEIGHTED_CONFIDENCE_THRESHOLD = 0.5

# Median-filter window (frames) for the IK fit's own output smoothing --
# unrelated to the gap detection above, this is purely a fit-quality
# step (per-frame IK is noisy), so it stays even though the old mismatch-
# based rejection this used to feed doesn't exist here anymore.
FILTERING_WINDOW_FRAMES = 5


def nanreduce_ignore_all_nan(func, x: np.ndarray, axis: int) -> np.ndarray:
    """`func` (`np.nanmin`/`np.nanmax`) over `x`, silencing the expected,
    harmless "All-NaN slice encountered" warning a frame with no detected
    instance at all (every keypoint NaN) triggers; `func` already returns
    NaN for it.
    """
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="All-NaN slice encountered")
        return func(x, axis=axis)


def leg_keypoint_indices(node_names: list[str]) -> list[int]:
    """Indices of `node_names`'s 30 leg keypoints (excludes "Th" and the 6
    non-leg nodes with no body-plan joint; see `build_sleap_to_joint_name_map`).
    """
    joint_map = build_sleap_to_joint_name_map()
    return [
        i for i, name in enumerate(node_names) if name in joint_map and name != "Th"
    ]


def median_filter_over_time(x: np.ndarray, window: int) -> np.ndarray:
    """Median-filter `x` along its leading (time) axis.

    Any trailing-axes "column" (e.g. one (node, coord) pair) that is NaN for
    every frame is left untouched, since `scipy.ndimage.median_filter` isn't
    NaN-aware and could otherwise fabricate finite output from a window that
    is entirely NaN.

    Args:
        x: `(period_length, ...)` array.
        window: Median filter window size, in frames. Values `<= 1` are a
            no-op.

    Returns:
        Array of the same shape as `x`.
    """
    if window <= 1:
        return x
    size = (window,) + (1,) * (x.ndim - 1)
    filtered = ndimage.median_filter(x, size=size, mode="nearest")
    all_nan = np.isnan(x).all(axis=0, keepdims=True)
    return np.where(all_nan, x, filtered)


def find_good_runs(good: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous `True` runs in `good`, as `(start, end)` (`end`
    exclusive), in chronological order. No closing/minimum-length -- a
    single bad frame between two good ones genuinely does split the solve
    into two independently warm-started stretches (see module docstring);
    there's no period output to tune away that edge case for anymore.
    """
    labeled, n_runs = ndimage.label(good)
    return [
        (int(idxs[0]), int(idxs[-1]) + 1)
        for run_id in range(1, n_runs + 1)
        for idxs in (np.flatnonzero(labeled == run_id),)
    ]


def report_mismatch_stats(
    pred_2d_mm: np.ndarray, fk_3d_mm: np.ndarray, leg_idxs: list[int]
) -> None:
    """Log fk-to-raw-prediction disagreement stats, purely informational
    (nothing here is rejected/dropped based on this -- see module
    docstring)."""
    dist = np.linalg.norm(pred_2d_mm[:, leg_idxs] - fk_3d_mm[:, leg_idxs, :2], axis=-1)
    frame_max = nanreduce_ignore_all_nan(np.nanmax, dist, axis=-1)
    n_attempted = int(np.isfinite(frame_max).sum())
    if n_attempted == 0:
        logger.warning("No frames had an IK attempt to report fk-to-pred mismatch for.")
        return
    logger.info(
        f"fk-to-pred mismatch (mm) over {n_attempted} attempted frames, "
        f"{len(leg_idxs)} leg keypoints: mean={np.nanmean(dist):.4f}, "
        f"mean of per-frame worst keypoint={np.nanmean(frame_max):.4f}"
    )


def main(
    input_path: Path,
    output_path: Path,
    with_ik: bool = True,
    body_plan_path: Path = DEFAULT_BODY_PLAN_PATH,
    neutral_weight: float = NEUTRAL_WEIGHT,
    override: bool = False,
) -> None:
    """Build `kinematics.h5` for one trial. See module docstring.

    Args:
        input_path: Dense per-frame pose2d `.h5` (see `spotlight_pose2d.
            io_utils.save_pose_h5`).
        output_path: Where to save `kinematics.h5`. Aborts if this already
            exists, unless `override` is set.
        with_ik: If False, only the `pose2d/` group is written (no
            inverse-kinematics fit at all).
        body_plan_path: Body-plan JSON (see
            `../flygym/scripts/export_model_for_quickik.py`).
        neutral_weight: `SequenceSolver`'s prior weight pulling the
            solution toward the body plan's neutral pose (see
            `spotlight_ik.neuromechfly.NEUTRAL_WEIGHT`'s comment for how
            the default was chosen).
        override: If True, overwrite `output_path` if it already exists.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    check_output_path(output_path, override)

    data = load_pose_h5(input_path)
    poses, keypoint_scores = data["poses"], data["keypoint_scores"]
    node_names, video_path = data["node_names"], data["video_path"]
    trial_dir = trial_dir_from_video_path(video_path)

    mapper = load_calibration_mapper(trial_dir)
    transform_matrices = load_transform_matrices(trial_dir)
    stage_positions_mm = load_stage_positions_mm(trial_dir)
    poses_mm = convert_px_to_mm(poses, transform_matrices, stage_positions_mm, mapper)
    poses_mm = poses_mm.astype(np.float32)

    ik_result = None
    if with_ik:
        body_plan = BodyPlan.load(body_plan_path)
        flipped_prob = load_flipped_prob(trial_dir)
        confidence = weighted_confidence(keypoint_scores, node_names)
        # A frame with no confidence at all (e.g. a hand-labeled/hand-
        # corrected frame with no score) reads NaN here, which correctly
        # never passes `>=` -- no separate NaN handling needed.
        good = (flipped_prob < FLIP_PROB_THRESHOLD) & (
            confidence >= WEIGHTED_CONFIDENCE_THRESHOLD
        )
        good_runs = find_good_runs(good)
        logger.info(
            f"{int(good.sum())}/{len(good)} frames good (flip < "
            f"{FLIP_PROB_THRESHOLD}, weighted confidence >= "
            f"{WEIGHTED_CONFIDENCE_THRESHOLD}) across {len(good_runs)} "
            "contiguous stretch(es); IK left NaN elsewhere"
        )

        n_frames, n_nodes = poses.shape[:2]
        n_dofs = len(body_plan.dof_names)
        dof_angles = np.full((n_frames, n_dofs), np.nan, dtype=np.float32)
        fk_3d_mm = np.full((n_frames, n_nodes, 3), np.nan, dtype=np.float32)
        fk_2d_px = np.full((n_frames, n_nodes, 2), np.nan, dtype=np.float32)

        for start, end in good_runs:
            sub_dof_angles, sub_fk_3d_mm = solve_period_ik(
                poses_mm[start:end], node_names, body_plan, neutral_weight
            )
            sub_dof_angles = median_filter_over_time(
                sub_dof_angles, FILTERING_WINDOW_FRAMES
            )
            sub_fk_3d_mm = median_filter_over_time(
                sub_fk_3d_mm, FILTERING_WINDOW_FRAMES
            )
            sub_fk_2d_px = convert_mm_to_px(
                sub_fk_3d_mm[..., :2],
                transform_matrices[start:end],
                stage_positions_mm[start:end],
                mapper,
            ).astype(np.float32)
            sub_fk_2d_px = median_filter_over_time(
                sub_fk_2d_px, FILTERING_WINDOW_FRAMES
            )

            dof_angles[start:end] = sub_dof_angles
            fk_3d_mm[start:end] = sub_fk_3d_mm
            fk_2d_px[start:end] = sub_fk_2d_px

        ik_result = InverseKinematicsResult(
            dof_names=body_plan.dof_names,
            dof_angles=dof_angles,
            keypoint_positions_3d_mm=fk_3d_mm,
            keypoint_positions_2d_px=fk_2d_px,
            neutral_weight=neutral_weight,
            keypoint_weight_scale=sleap_keypoint_weight_scale(),
        )
        report_mismatch_stats(poses_mm, fk_3d_mm, leg_keypoint_indices(node_names))

    save_kinematics_h5(
        output_path,
        node_names=node_names,
        keypoint_positions_2d_px=poses,
        keypoint_positions_2d_mm=poses_mm,
        keypoint_positions_confidence=keypoint_scores,
        inverse_kinematics=ik_result,
    )
    logger.info(f"Saved {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
