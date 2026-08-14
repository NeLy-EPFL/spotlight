"""Fit inverse kinematics (QuickIK) to one trial's dense 2D pose predictions
(`pose2d.h5`, see `pose2d.io_utils.save_pose_h5`) and save the fit as
`inverse_kinematics.h5` (see `invkin.io_utils`'s module docstring): one row
per frame of the whole trial, no period/segment concept. Curating "which
stretches are good enough for, e.g., FlyGym snippet selection" happens
downstream, using this file's own per-frame confidence/mismatch data, not a
pre-baked accept/reject decision made here.

IK is skipped (left NaN) for frames the localization model flagged flipped
(`flipped_prob >= flip_confidence_threshold`) or where the 2D pose's own
weighted confidence (`localization.flip_label.weighted_confidence`: the
SAME proxy used to pseudo-label that model's own training data) is below
`min_confidence`: both indicate the fly's own crop/orientation is unreliable
enough that fitting IK to it isn't meaningful, independent of any downstream
mismatch-based quality check. This is a per-frame attempt/skip decision, not
exposed as a period in the output: IK still runs one contiguous good
stretch at a time internally (QuickIK's `SequenceSolver` warm-starts
frame-to-frame, so a stretch's solve shouldn't warm-start off an unrelated
stretch across a skipped run; see `solve_period_ik`'s own docstring), but
those stretch boundaries are never written anywhere.

Separately, `max_mismatch` (how far the FK result ends up from the original
2D prediction) gates `mismatch_mask`, a per-frame display-acceptance mask
saved alongside the fit for the QA video to use directly (see
`invkin.io_utils`'s module docstring): this never drops data from the fit
itself, only marks which frames are trustworthy enough to show.

Fits the NeuroMechFly body plan (see `../flygym/scripts/
export_model_for_quickik.py`) via QuickIK's `SequenceSolver` with an
orthographic XY projection (see `invkin.neuromechfly.
solve_period_ik`). Only the 30 leg SLEAP nodes are used as real
observations; "Th" (thorax) never is (see `solve_period_ik`'s own
comment: it's the floating root, not a real keypoint on the body plan).
"""

import warnings
from pathlib import Path

import numpy as np
from loguru import logger
from scipy import ndimage

from spotlight.postprocessing.common.smoothing import (
    seconds_to_odd_frames,
    smooth_acceptance_mask,
)
from spotlight.postprocessing.invkin.mapping import (
    convert_mm_to_px,
    convert_px_to_mm,
    load_calibration_mapper,
    load_control_freq_hz,
    load_flipped_prob,
    load_stage_positions_mm,
    load_transform_matrices,
)
from spotlight.postprocessing.invkin.io_utils import (
    InverseKinematicsResult,
    save_inverse_kinematics_h5,
)
from spotlight.postprocessing.invkin.neuromechfly import (
    NEUTRAL_WEIGHT,
    BodyPlan,
    build_sleap_to_joint_name_map,
    sleap_keypoint_weight_scale,
    solve_period_ik,
)
from spotlight.postprocessing.localization.flip_label import weighted_confidence
from spotlight.postprocessing.pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    trial_dir_from_video_path,
)
from spotlight.common import get_assets_dir

DEFAULT_BODY_PLAN_PATH = (
    get_assets_dir() / "inverse_kinematics" / "neuromechfly_ypr_legs.json"
)

# Fit-quality smoothing (frames), unrelated to the flip/confidence gap
# detection below: per-frame IK is noisy, so this stays a fixed constant
# rather than a CLI knob (it's not something users have asked to tune).
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
    exclusive), in chronological order. No closing/minimum-length: a
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


def compute_mismatch_mask(
    ik_attempted: np.ndarray,
    pred_2d_mm: np.ndarray,
    fk_3d_mm: np.ndarray,
    leg_idxs: list[int],
    max_mismatch: float,
    denoise_window: int,
) -> np.ndarray:
    """Per-frame display-acceptance mask: IK was attempted and its
    fk-to-prediction mismatch (mm, worst leg keypoint) is within
    `max_mismatch`, denoised over `denoise_window` frames (see
    `common.smoothing.smooth_acceptance_mask`). Also logs the raw
    (undenoised) mismatch stats, purely informational.
    """
    dist = np.linalg.norm(pred_2d_mm[:, leg_idxs] - fk_3d_mm[:, leg_idxs, :2], axis=-1)
    frame_max_mismatch = nanreduce_ignore_all_nan(np.nanmax, dist, axis=-1)
    n_attempted = int(np.isfinite(frame_max_mismatch).sum())
    if n_attempted == 0:
        logger.warning("No frames had an IK attempt to report fk-to-pred mismatch for.")
    else:
        logger.info(
            f"fk-to-pred mismatch (mm) over {n_attempted} attempted frames, "
            f"{len(leg_idxs)} leg keypoints: mean={np.nanmean(dist):.4f}, "
            f"mean of per-frame worst keypoint={np.nanmean(frame_max_mismatch):.4f}"
        )
    raw_ok = ik_attempted & (frame_max_mismatch <= max_mismatch)
    return smooth_acceptance_mask(raw_ok, denoise_window)


def solve_ik(
    input_path: Path,
    output_path: Path,
    body_plan_path: Path = DEFAULT_BODY_PLAN_PATH,
    neutral_weight: float = NEUTRAL_WEIGHT,
    upright_prior_weight: float = 0.1,
    flip_confidence_threshold: float = 0.5,
    min_confidence: float = 0.5,
    max_mismatch: float = 0.3,
    mismatch_denoise_window_sec: float = 0.05,
    frame_start: int = 0,
    override: bool = False,
) -> None:
    """Build `inverse_kinematics.h5` for one trial. See module docstring.

    Args:
        input_path: Dense per-frame `pose2d.h5` (see `pose2d.io_utils.
            save_pose_h5`).
        output_path: Where to save `inverse_kinematics.h5`. Aborts if this
            already exists, unless `override` is set.
        body_plan_path: Body-plan JSON (see
            `../flygym/scripts/export_model_for_quickik.py`).
        neutral_weight: `SequenceSolver`'s prior weight pulling the
            solution toward the body plan's neutral pose (see
            `invkin.neuromechfly.NEUTRAL_WEIGHT`'s comment for how
            the default was chosen).
        upright_prior_weight: Reserved for a future upright-body prior;
            unused for now (`solve_period_ik` doesn't yet support one).
        flip_confidence_threshold: A frame is skipped for IK if the
            localization model's own flip probability is at or above this.
        min_confidence: A frame is skipped for IK if its weighted keypoint
            confidence is below this.
        max_mismatch: Max fk-to-2D-prediction mismatch (mm, worst leg
            keypoint) for `mismatch_mask` to accept a frame.
        mismatch_denoise_window_sec: `mismatch_mask` is denoised (binary
            open then close) over this many seconds, converted to an odd
            frame count using the trial's own `behavior_fps`.
        frame_start: `input_path`'s own frame 0 is this many frames into
            the trial (see `--frame-range`): `behavior_frames_metadata.csv`
            (unlike `input_path` and `behavior_alignment_transforms.h5`,
            both already trimmed by whichever upstream stage produced
            them) always covers the whole trial, so stage positions need
            this offset to line up with `input_path`'s own rows.
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
    stage_positions_mm = load_stage_positions_mm(trial_dir)[
        frame_start : frame_start + len(poses)
    ]
    poses_mm = convert_px_to_mm(poses, transform_matrices, stage_positions_mm, mapper)
    poses_mm = poses_mm.astype(np.float32)

    behavior_fps = load_control_freq_hz(trial_dir)
    flipped_prob = load_flipped_prob(trial_dir)
    confidence = weighted_confidence(keypoint_scores, node_names)
    # A frame with no confidence at all (e.g. a hand-labeled/hand-corrected
    # frame with no score) reads NaN here, which correctly never passes
    # `>=`; no separate NaN handling needed.
    good = (flipped_prob < flip_confidence_threshold) & (confidence >= min_confidence)
    good_runs = find_good_runs(good)
    logger.info(
        f"{int(good.sum())}/{len(good)} frames good (flip < "
        f"{flip_confidence_threshold}, weighted confidence >= "
        f"{min_confidence}) across {len(good_runs)} contiguous stretch(es); "
        "IK left NaN elsewhere"
    )

    body_plan = BodyPlan.load(body_plan_path)
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
        sub_fk_3d_mm = median_filter_over_time(sub_fk_3d_mm, FILTERING_WINDOW_FRAMES)
        sub_fk_2d_px = convert_mm_to_px(
            sub_fk_3d_mm[..., :2],
            transform_matrices[start:end],
            stage_positions_mm[start:end],
            mapper,
        ).astype(np.float32)
        sub_fk_2d_px = median_filter_over_time(sub_fk_2d_px, FILTERING_WINDOW_FRAMES)

        dof_angles[start:end] = sub_dof_angles
        fk_3d_mm[start:end] = sub_fk_3d_mm
        fk_2d_px[start:end] = sub_fk_2d_px

    ik_attempted = ~np.isnan(dof_angles).any(axis=-1)
    mismatch_denoise_window = seconds_to_odd_frames(
        mismatch_denoise_window_sec, behavior_fps
    )
    mismatch_mask = compute_mismatch_mask(
        ik_attempted, poses_mm, fk_3d_mm, leg_keypoint_indices(node_names),
        max_mismatch, mismatch_denoise_window,
    )  # fmt: skip

    result = InverseKinematicsResult(
        dof_names=body_plan.dof_names,
        dof_angles=dof_angles,
        keypoint_positions_3d_mm=fk_3d_mm,
        keypoint_positions_2d_px=fk_2d_px,
        mismatch_mask=mismatch_mask,
        neutral_weight=neutral_weight,
        keypoint_weight_scale=sleap_keypoint_weight_scale(),
        max_mismatch=max_mismatch,
        mismatch_denoise_window_sec=mismatch_denoise_window_sec,
    )
    save_inverse_kinematics_h5(output_path, node_names=node_names, result=result)
    logger.info(f"Saved {output_path}")
