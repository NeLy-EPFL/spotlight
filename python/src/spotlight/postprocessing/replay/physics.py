"""Replay one trial's solved IK joint angles (`inverse_kinematics.h5`, see
`invkin.io_utils`) in FlyGym (CPU physics) and save the rendered frames as
`physics_replay.h5` (see `replay.io`'s module docstring), for the QA
video's row-2 replay panel. A persisted pipeline stage in its own right
(unlike the QA video, which just reads this back), so re-tuning the physics
knobs below doesn't require re-solving IK or re-running alignment/pose2d.

See `replay.flygym`'s own module docstring for what "replay" means
here (position-actuator-driven physics, not a kinematic bypass) and for the
camera/restart-on-fall mechanics.
"""

from pathlib import Path
from typing import Literal

import numpy as np
from flygym.compose import ActuatorType
from joblib import Parallel, delayed
from loguru import logger

from spotlight.postprocessing.common.parallel import resolve_num_workers
from spotlight.postprocessing.common.smoothing import (
    seconds_to_frames,
    smooth_unit_vectors,
)
from spotlight.postprocessing.replay.flygym import (
    DEFAULT_ACTUATOR_GAIN,
    DEFAULT_ADHESION_GAIN,
    DEFAULT_FALL_TILT_THRESHOLD_DEG,
    ORBIT_CAM_AZIMUTH_OFFSET_DEG,
    DEFAULT_TARSAL_STIFFNESS,
    build_dof_reorder_index,
    build_fly_and_world,
    record_period_yaw_in_worker,
    replay_period_in_worker,
)
from spotlight.postprocessing.invkin.io_utils import load_inverse_kinematics_h5
from spotlight.postprocessing.invkin.qa_clips import find_ik_runs
from spotlight.postprocessing.replay.io import (
    PhysicsReplayResult,
    save_physics_replay_h5,
)
from spotlight.postprocessing.pose2d.io_utils import check_output_path
from spotlight.postprocessing.visualize import PANEL_SIZE


def _smoothed_camera_azimuth_rad(yaw_rad: np.ndarray, sigma: float) -> np.ndarray:
    """This period's camera azimuth (radians, including
    `ORBIT_CAM_AZIMUTH_OFFSET_DEG`), smoothing `yaw_rad` (the thorax's own
    raw per-step heading, see `record_period_yaw`) across the period first.
    Smooths the heading's unit-vector components, not the raw angle, to
    sidestep the wraparound a naive angle average would hit near +/-180
    degrees."""
    vectors = np.stack([np.cos(yaw_rad), np.sin(yaw_rad)], axis=1)
    smoothed = smooth_unit_vectors(vectors, sigma)
    smoothed_yaw = np.arctan2(smoothed[:, 1], smoothed[:, 0])
    return smoothed_yaw + np.radians(ORBIT_CAM_AZIMUTH_OFFSET_DEG)


def replay_physics(
    input_path: Path,
    output_path: Path,
    behavior_fps: float,
    actuator_gain: float = DEFAULT_ACTUATOR_GAIN,
    adhesion_gain: float = DEFAULT_ADHESION_GAIN,
    tarsal_stiffness: float = DEFAULT_TARSAL_STIFFNESS,
    fall_tilt_threshold_deg: float = DEFAULT_FALL_TILT_THRESHOLD_DEG,
    viz_heading_denoise_sigma_sec: float = 0.015,
    num_workers: int | Literal["auto"] = "auto",
    override: bool = False,
) -> None:
    """Build `physics_replay.h5` for one trial. See module docstring.

    Args:
        input_path: `inverse_kinematics.h5` (see `invkin.io_utils`).
        output_path: Where to save `physics_replay.h5`. Aborts if this
            already exists, unless `override` is set.
        behavior_fps: The trial's recording frame rate; replay's control
            loop runs at this rate (zero-order hold between recorded frames).
        actuator_gain: Position-actuator gain (uN*mm/rad) on the leg DOFs.
        adhesion_gain: Leg adhesion force (uN).
        tarsal_stiffness: Passive spring stiffness of the (unactuated)
            tarsal joints.
        fall_tilt_threshold_deg: Restart the physics state once the
            thorax tips this many degrees from vertical.
        viz_heading_denoise_sigma_sec: Display-only: Gaussian smoothing
            (seconds, converted to frames via `behavior_fps`) applied to
            the replay camera's own yaw-tracking, so keypoint/physics
            noise doesn't jitter it: never touches the physics itself,
            only which way the camera looks. Costs a second, renderer-less
            physics pass per period (see `record_period_yaw`) to know each
            step's heading before rendering the real, camera-bearing pass.
            -1 disables smoothing (and that second pass).
        num_workers: One IK period per task, across this many joblib
            workers. `"auto"` uses `$SLURM_CPUS_PER_TASK` if set, else all
            cores (see `common.parallel.resolve_num_workers`).
        override: If True, overwrite `output_path` if it already exists.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    check_output_path(output_path, override)

    ik = load_inverse_kinematics_h5(input_path)
    n_frames = len(ik.dof_angles)
    periods = find_ik_runs(ik.dof_angles)
    logger.info(f"Replaying {len(periods)} IK period(s) in FlyGym...")

    # Only used here for its DOF ordering (fly/world/orbit_cam themselves
    # aren't simulated in this process): each parallel worker builds its own
    # via `replay_period_in_worker`, since these aren't picklable across
    # process boundaries.
    probe_fly, _, _ = build_fly_and_world(
        actuator_gain=actuator_gain,
        adhesion_gain=adhesion_gain,
        tarsal_stiffness=tarsal_stiffness,
    )
    actuated_dof_order = probe_fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    reorder_index = build_dof_reorder_index(ik.dof_names, actuated_dof_order)

    physics_kwargs = dict(
        fall_tilt_threshold_deg=fall_tilt_threshold_deg,
        actuator_gain=actuator_gain,
        adhesion_gain=adhesion_gain,
        tarsal_stiffness=tarsal_stiffness,
    )

    camera_azimuth_rad_by_period = [None] * len(periods)
    sigma = seconds_to_frames(viz_heading_denoise_sigma_sec, behavior_fps)
    if sigma != -1:
        logger.info(
            f"Recording camera heading for {len(periods)} period(s) "
            "(renderer-less dry run, before the real replay pass)..."
        )
        yaw_pool = Parallel(n_jobs=resolve_num_workers(num_workers))
        yaw_by_period = yaw_pool(
            delayed(record_period_yaw_in_worker)(
                ik.dof_angles[start:end][:, reorder_index], behavior_fps, **physics_kwargs
            )
            for start, end in periods
        )  # fmt: skip
        camera_azimuth_rad_by_period = [
            _smoothed_camera_azimuth_rad(yaw_rad, sigma) for yaw_rad in yaw_by_period
        ]

    replay_pool = Parallel(
        n_jobs=resolve_num_workers(num_workers), return_as="generator"
    )
    rendered_by_period = replay_pool(
        delayed(replay_period_in_worker)(
            ik.dof_angles[start:end][:, reorder_index],
            behavior_fps,
            PANEL_SIZE,
            camera_azimuth_rad=camera_azimuth_rad,
            **physics_kwargs,
        )
        for (start, end), camera_azimuth_rad in zip(periods, camera_azimuth_rad_by_period)
    )  # fmt: skip

    total_rendered = sum(end - start for start, end in periods)
    frames = np.empty((total_rendered, PANEL_SIZE, PANEL_SIZE, 3), dtype=np.uint8)
    frame_lookup = np.full(n_frames, -1, dtype=np.int64)
    next_idx = 0
    log_every = max(1, len(periods) // 20)
    for n_done, ((start, end), rendered) in enumerate(
        zip(periods, rendered_by_period), start=1
    ):
        frames[next_idx : next_idx + len(rendered)] = rendered
        frame_lookup[start:end] = np.arange(next_idx, next_idx + len(rendered))
        next_idx += len(rendered)
        if n_done % log_every == 0 or n_done == len(periods):
            logger.info(f"Replayed {n_done}/{len(periods)} IK periods...")

    save_physics_replay_h5(
        output_path,
        PhysicsReplayResult(
            frames=frames,
            frame_lookup=frame_lookup,
            actuator_gain=actuator_gain,
            adhesion_gain=adhesion_gain,
            tarsal_stiffness=tarsal_stiffness,
            fall_tilt_threshold_deg=fall_tilt_threshold_deg,
        ),
    )
    logger.info(f"Saved {output_path}")
