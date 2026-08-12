#!/usr/bin/env python
"""Find good prediction periods and fit inverse kinematics (QuickIK) to one
trial's dense pose `.h5`.

Single trial. Input is the shared dense per-frame `.h5` (see
`spotlight_pose2d.io_utils.save_pose_h5`, e.g. one of `run_inference_final.
sh`'s `*_pose2d_final_predictions.h5`), in the aligned pixel domain.

1. Finds contiguous "good" periods from a confidence mask: a frame is
   accepted if every one of its 30 leg keypoints has a SLEAP `keypoint_
   scores` >= `--min-confidence`. Binary closing with a `--closing-size`
   -frame structuring element bridges short gaps, then only runs at least
   `--min-period-length` frames long are kept (see `spotlight_ik.periods.
   find_periods`). `is_label` isn't used here: this tool's real input is
   exhaustive production inference over every frame, where genuine hand
   labels are a negligible fraction, unlike the small hand-labeled batches
   `is_label`-based period-finding was designed for.
2. Converts each period's aligned-pixel `poses` to physical (mm) coordinates
   (needs, from the input's own linked video's trial directory: the
   per-frame raw-to-aligned affine transforms, per-frame stage position, and
   camera calibration -- see `spotlight_ik.calibration`).
3. Fits the NeuroMechFly body plan (see
   `../flygym/scripts/export_model_for_quickik.py`) to that mm keypoint
   sequence via QuickIK's `SequenceSolver` with an orthographic XY
   projection (see `spotlight_ik.neuromechfly.solve_period_ik`). Only the
   31 SLEAP nodes with a corresponding body-plan joint (30 leg nodes plus
   "Th" -> "thorax") are used as observations.
4. Median-filters `ik_dofangles_rad`/`fk_3d_mm`/`fk_2d_px` over time (window
   `--filtering-mask-frames`), then checks each frame's fit against its own
   observation: the xy distance between `pred_2d_mm` and (filtered)
   `fk_3d_mm` (dropping z), and rejects a frame if its worst keypoint
   exceeds `--max-mismatch` (mm). This refined accepted mask is run back
   through `find_periods` (closing size and minimum length both
   `--filtering-mask-frames`/`--min-period-length`), so a period from step 1
   can end up dropped, shortened, or split into several.
5. Rejects frames where the fly is holding still: for each remaining
   sub-period, computes each leg's own joint-angle excursion (the largest
   range any single one of its DOFs sweeps through within a
   `--movement-window-ms` window centered on that frame; see
   `spotlight_ik.periods.compute_joint_excursion_deg` for why a windowed
   range, not a frame-to-frame derivative), and keeps only frames where the
   largest excursion, over every leg, is at least `--min-joint-excursion-
   deg`. This deliberately doesn't care whether the fly is translating
   through space -- grooming or otherwise moving legs while stationary
   still counts as "not standing still" -- only whether its joints are
   moving at all. Run through `find_periods` again (closing size
   `--filtering-mask-frames`, not the excursion window itself -- that
   window is sized for smoothing, and would erase any sub-period shorter
   than it via `binary_closing`'s edge effects; minimum length
   `--min-period-length`), so a sub-period can again end up dropped,
   shortened, or split.

Saves one `.h5` with a group per re-segmented period (`pred_2d_px`,
`pred_2d_mm`, `confidence`, `ik_dofangles_rad`, `fk_3d_mm`, `fk_2d_px`; the
last three independently median-filtered, so may be slightly inconsistent
with each other, e.g. `fk_2d_px` not an exact projection of `fk_3d_mm` --
expected and acceptable here). See `spotlight_ik.io_utils.save_ikfk_h5`.

Usage:
    python tools/spotlight_ik/solve_ik.py \\
        --input-path trial_pose2d_final_predictions.h5 \\
        --output-path trial_ikfk.h5
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
    load_control_freq_hz,
    load_stage_positions_mm,
    load_transform_matrices,
)
from spotlight_postprocessing.spotlight_ik.io_utils import Period, save_ikfk_h5
from spotlight_postprocessing.spotlight_ik.neuromechfly import (
    NEUTRAL_WEIGHT,
    BodyPlan,
    build_sleap_to_joint_name_map,
    solve_period_ik,
)
from spotlight_postprocessing.spotlight_ik.periods import (
    compute_joint_excursion_deg,
    find_periods,
)
from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    trial_dir_from_video_path,
)
from spotlight_tools.common import get_assets_dir

DEFAULT_BODY_PLAN_PATH = get_assets_dir() / "neuromechfly_ypr_legs.json"


def nanreduce_ignore_all_nan(func, x: np.ndarray, axis: int) -> np.ndarray:
    """`func` (`np.nanmin`/`np.nanmax`) over `x`, silencing the expected,
    harmless "All-NaN slice encountered" warning a frame with no detected
    instance at all (every keypoint NaN) triggers; `func` already returns
    NaN for it, which is the correct "never accepted" result downstream.
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


def compute_mismatch_mm(
    pred_2d_mm: np.ndarray, fk_3d_mm: np.ndarray, leg_idxs: list[int]
) -> np.ndarray:
    """Per-frame worst-leg-keypoint xy mismatch between predictions and FK, in mm.

    Args:
        pred_2d_mm: `(period_length, n_nodes, 2)` physical (mm) keypoints.
        fk_3d_mm: `(period_length, n_nodes, 3)` FK result, in the same frame
            as `pred_2d_mm`.
        leg_idxs: See `leg_keypoint_indices`; "Th" and the 6 non-leg nodes
            are never a valid FK comparison point (see `solve_period_ik`)
            and are excluded.

    Returns:
        `(period_length,)` max, over leg keypoints, of the xy distance
        between `pred_2d_mm` and `fk_3d_mm`.
    """
    dist = np.linalg.norm(pred_2d_mm[:, leg_idxs] - fk_3d_mm[:, leg_idxs, :2], axis=-1)
    return nanreduce_ignore_all_nan(np.nanmax, dist, axis=-1)


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


def report_mismatch_stats(periods: list[Period], leg_idxs: list[int]) -> None:
    """Log fk-to-raw-prediction disagreement stats across every re-segmented period."""
    if not periods:
        logger.warning("No periods to report fk-to-pred mismatch stats for.")
        return

    dist = np.concatenate(
        [
            np.linalg.norm(
                period.pred_2d_mm[:, leg_idxs] - period.fk_3d_mm[:, leg_idxs, :2],
                axis=-1,
            )
            for period in periods
        ],
        axis=0,
    )  # (total_frames, n_leg_keypoints)
    frame_max = nanreduce_ignore_all_nan(np.nanmax, dist, axis=-1)
    logger.info(
        f"fk-to-pred mismatch (mm) over {dist.shape[0]} frames, "
        f"{len(leg_idxs)} leg keypoints: mean={np.nanmean(dist):.4f}, "
        f"mean of per-frame worst keypoint={np.nanmean(frame_max):.4f}"
    )


def main(
    input_path: Path,
    output_path: Path,
    body_plan_path: Path = DEFAULT_BODY_PLAN_PATH,
    min_confidence: float = 0.3,
    closing_size: int = 5,
    min_period_length: int = 30,
    max_mismatch: float = 0.3,
    filtering_mask_frames: int = 5,
    min_joint_excursion_deg: float = 10.0,
    movement_window_ms: float = 100.0,
    neutral_weight: float = NEUTRAL_WEIGHT,
    override: bool = False,
) -> None:
    """Find good periods and fit IK to one trial's `.h5`. See module docstring.

    Args:
        input_path: Dense per-frame `.h5` (see `spotlight_pose2d.io_utils.
            save_pose_h5`).
        output_path: Where to save the periods+IK/FK `.h5`. Aborts if this
            already exists, unless `override` is set.
        body_plan_path: Body-plan JSON (see
            `../flygym/scripts/export_model_for_quickik.py`).
        min_confidence: Minimum per-frame, worst-leg-keypoint SLEAP
            confidence for a frame to seed an initial period. Empirically
            tunable; check the effect visually via `make_videos.py`.
        closing_size: Structuring element size for closing the initial
            confidence mask before finding periods (see `find_periods`).
        min_period_length: Minimum length, in frames, for a period (initial
            or re-segmented) to be kept.
        max_mismatch: Maximum tolerated xy mismatch (mm) between `pred_2d_mm`
            and `fk_3d_mm` for a frame's worst leg keypoint; frames
            exceeding this are rejected (see `compute_mismatch_mm`).
        filtering_mask_frames: Structuring element size for closing the
            refined accepted mask, and window size for median-filtering
            `ik_dofangles_rad`/`fk_3d_mm`/`fk_2d_px` over time.
        min_joint_excursion_deg: Minimum per-frame joint excursion (see
            `spotlight_ik.periods.compute_joint_excursion_deg`) for a frame
            to count as the fly moving rather than holding still.
            Empirically grounded (not just guessed): genuinely held-still
            moments read a few degrees at a 100ms window, real movement
            (walking or grooming) tens of degrees, with a fairly continuous
            climb in between rather than a sharp gap -- 10.0 was chosen to
            additionally exclude the still-fairly-subtle 5-10 degree band
            (raised from an initial, more conservative 5.0 default, which
            only excluded a genuinely-still-or-near-still 0-5 degree band).
        movement_window_ms: Window size (converted to frames via this
            trial's own recording rate), in milliseconds, for the joint-
            excursion check above.
        neutral_weight: `SequenceSolver`'s prior weight pulling the solution
            toward the body plan's neutral pose (see `spotlight_ik.
            neuromechfly.NEUTRAL_WEIGHT`'s comment for how the default was
            chosen).
        override: If True, overwrite `output_path` if it already exists.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    check_output_path(output_path, override)

    data = load_pose_h5(input_path)
    poses, keypoint_scores = data["poses"], data["keypoint_scores"]
    node_names, video_path = data["node_names"], data["video_path"]
    trial_dir = trial_dir_from_video_path(video_path)
    leg_idxs = leg_keypoint_indices(node_names)

    body_plan = BodyPlan.load(body_plan_path)
    mapper = load_calibration_mapper(trial_dir)
    transform_matrices = load_transform_matrices(trial_dir)
    stage_positions_mm = load_stage_positions_mm(trial_dir)
    control_freq_hz = load_control_freq_hz(trial_dir)
    movement_window_frames = max(1, round(movement_window_ms / 1000 * control_freq_hz))

    poses_mm = convert_px_to_mm(poses, transform_matrices, stage_positions_mm, mapper)
    poses_mm = poses_mm.astype(np.float32)

    frame_min_confidence = nanreduce_ignore_all_nan(
        np.nanmin, keypoint_scores[:, leg_idxs], axis=-1
    )
    accepted_initial = frame_min_confidence >= min_confidence
    initial_periods = find_periods(accepted_initial, closing_size, min_period_length)
    logger.info(f"{len(initial_periods)} initial confidence-based periods")

    new_periods = []
    n_mismatch_periods = 0
    for start_idx, end_idx in initial_periods:
        pred_2d_px = poses[start_idx:end_idx]
        pred_2d_mm = poses_mm[start_idx:end_idx]
        confidence = keypoint_scores[start_idx:end_idx]
        transforms = transform_matrices[start_idx:end_idx]
        stage_pos = stage_positions_mm[start_idx:end_idx]

        dof_angles, fk_3d_mm = solve_period_ik(
            pred_2d_mm, node_names, body_plan, neutral_weight
        )
        fk_2d_px = convert_mm_to_px(
            fk_3d_mm[..., :2], transforms, stage_pos, mapper
        ).astype(np.float32)

        # Median-filter BEFORE checking the mismatch criterion (not after),
        # so `max_mismatch` bounds the quality of what's actually stored:
        # filtering after the check could let a frame drift back out past
        # the threshold that supposedly gated its acceptance.
        dof_angles = median_filter_over_time(dof_angles, filtering_mask_frames)
        fk_3d_mm = median_filter_over_time(fk_3d_mm, filtering_mask_frames)
        fk_2d_px = median_filter_over_time(fk_2d_px, filtering_mask_frames)

        mismatch = compute_mismatch_mm(pred_2d_mm, fk_3d_mm, leg_idxs)
        accepted_refined = mismatch <= max_mismatch
        sub_periods = find_periods(
            accepted_refined, filtering_mask_frames, min_period_length
        )
        n_mismatch_periods += len(sub_periods)

        for local_start, local_end in sub_periods:
            sl = slice(local_start, local_end)
            sub_dof_angles = dof_angles[sl]

            excursion_deg = compute_joint_excursion_deg(
                sub_dof_angles, body_plan.dof_names, movement_window_frames
            )
            moving = excursion_deg >= min_joint_excursion_deg
            # `filtering_mask_frames` (not `movement_window_frames`) as the
            # closing size here: `binary_closing`'s erosion step treats
            # out-of-bounds neighbors as False, so closing with a window
            # larger than a period is long erases the whole period
            # regardless of its content. `movement_window_frames` (100ms)
            # is sized for smoothing the excursion signal, not for this.
            movement_periods = find_periods(
                moving, filtering_mask_frames, min_period_length
            )

            for m_start, m_end in movement_periods:
                msl = slice(m_start, m_end)
                new_periods.append(
                    Period(
                        start_idx=start_idx + local_start + m_start,
                        end_idx=start_idx + local_start + m_end,
                        pred_2d_px=pred_2d_px[sl][msl],
                        pred_2d_mm=pred_2d_mm[sl][msl],
                        confidence=confidence[sl][msl],
                        ik_dofangles_rad=sub_dof_angles[msl],
                        fk_3d_mm=fk_3d_mm[sl][msl],
                        fk_2d_px=fk_2d_px[sl][msl],
                    )
                )

    save_ikfk_h5(
        output_path,
        new_periods,
        node_names,
        body_plan.dof_names,
        video_path,
        extra_attrs={
            "min_confidence": min_confidence,
            "closing_size": closing_size,
            "max_mismatch_mm": max_mismatch,
            "filtering_mask_frames": filtering_mask_frames,
            "min_joint_excursion_deg": min_joint_excursion_deg,
            "movement_window_ms": movement_window_ms,
            "min_period_length": min_period_length,
            "neutral_weight": neutral_weight,
        },
    )
    logger.info(
        f"{len(initial_periods)} confidence-based -> {n_mismatch_periods} "
        f"mismatch-refined -> {len(new_periods)} movement-refined periods; "
        f"saved to {output_path}"
    )
    report_mismatch_stats(new_periods, leg_idxs)


if __name__ == "__main__":
    tyro.cli(main)
