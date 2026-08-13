"""Replay solved IK joint angles in FlyGym (CPU physics) for the QA video's
row-2 replay panel (see `postprocessing.visualize`).

FlyGym has no kinematic-teleport mode: joints are only ever driven through
position actuators (a PD controller), which then drive the physics
simulation -- so this is a physics replay, not a bypass of it. In
particular, the root (thorax free joint) is never actuated: its pose
emerges entirely from gravity, ground contact, and leg-adhesion reaction
forces, since the IK fit never observes it directly either (see
`invkin.neuromechfly`'s module docstring).

The fly/world setup and DOF reorder below are ported from `poseforge2`'s own
`flygym_replay.py` (`poseforge2/src/poseforge/motion_prior/spotlight_ik/
flygym_replay.py`, itself ported from `prior2d/scripts/flygym_replay.py`),
using the same tuned CPU-replay values (actuator gain, adhesion, tarsal
stiffness) validated there against this same QuickIK-solved body plan.

The camera and restart-on-fall logic are ported from `poseforge`'s own
`production/spotlight/neuromechfly_replay.py`: the camera's world pose is
computed explicitly every physics step and written directly into
`mj_data.cam_xpos`/`cam_xmat` (not a mocap-driven child camera or FlyGym's
own tracking-camera modes, both of which either stay world-fixed or couple
orientation to the fly's full pose, pitch/roll included), so its height,
elevation, and roll stay exactly fixed and only its azimuth (about world Z)
tracks the fly's own heading -- the fly's apparent orientation never
changes on screen, only the ground/background moves as it walks, matching
this file's own `visualize.py` panel's 3D-reconstruction camera. If the fly
tips over past `fall_tilt_threshold_deg`, its physics state (not the
renderer, and not the control loop's own clock) is reset to the spawn pose
and adhesion is re-armed, then the replay continues feeding the same,
still-advancing target-angle sequence.
"""

import logging

import numpy as np
import mujoco as mj
from flygym import Simulation
from flygym.anatomy import (
    ActuatedDOFPreset,
    AxisOrder,
    BodySegment,
    JointDOF,
    JointPreset,
    Skeleton,
)
from flygym.compose import (
    ActuatorType,
    FlatGroundWorld,
    KinematicPosePreset,
    NeuroMechFly,
)
from flygym.utils.math import Rotation3D

logger = logging.getLogger(__name__)

# FlyGym's own default (see mujoco_globals.yaml); a much coarser physics
# step makes the contact-rich leg/ground dynamics numerically unstable.
PHYSICS_TIMESTEP = 1e-4  # seconds

SPAWN_POSITION = (0.0, 0.0, 0.7)  # thorax center, mm above the ground
SPAWN_ROTATION = Rotation3D(format="quat", values=(1.0, 0.0, 0.0, 0.0))

# Tunable CPU-replay values (see module docstring), validated against this same
# QuickIK-solved body plan in both prior2d's and poseforge2's own
# FlyGym replay tools, and in poseforge's SeqIKPy-based one.
DEFAULT_ACTUATOR_GAIN = 75.0  # uN*mm/rad (FlyGym's own "kp" position-actuator gain)
DEFAULT_ADHESION_GAIN = 12.0  # uN
DEFAULT_TARSAL_STIFFNESS = 8.0

THORAX_SEGMENT = BodySegment("c_thorax")

# Restart-on-fall: tip past this many degrees from vertical (measured from
# the thorax's own up-vector) and the physics state is reset (see
# `restart_physics`).
DEFAULT_FALL_TILT_THRESHOLD_DEG = 60.0
DEFAULT_MAX_RESTARTS_PER_PERIOD = 5

# Orbiting camera: same elevation as `visualize.py`'s own synthetic-3D
# panel (`compute_camera_orientation`/`draw_fk_3d_panel`); distance and
# azimuth offset picked by rendering a sweep and comparing by eye (see
# `poseforge`'s own `neuromechfly_replay.py`, where these were first tuned).
ORBIT_CAM_DISTANCE_MM = 12.0
ORBIT_CAM_ELEVATION_DEG = 30.0
ORBIT_CAM_FOVY_DEG = 30.0
ORBIT_CAM_AZIMUTH_OFFSET_DEG = 220.0

# Ground checker floor: `FlatGroundWorld`'s own default (4 mm/square, from
# its default half_size=1000 mm and texrepeat=250) is coarser than the IK
# 3D panel's own 1 mm grid (`make_videos.GRID_SPACING_MM`) -- matched here
# for visual consistency between the two side-by-side panels.
GROUND_CHECKER_SQUARE_MM = 1.0


def build_dof_reorder_index(
    dof_names: list[str], actuated_dof_order: list[JointDOF]
) -> np.ndarray:
    """Map body-plan DOF order to the order FlyGym's actuators expect.

    Both use the same `"{parent}-{child}-{axis}"` naming (verified to align
    exactly, including sign convention, since the body plan was exported
    from this same FlyGym model; see
    `flygym/scripts/export_model_for_quickik.py`).

    Args:
        dof_names: DOF names in `ik_dofangles_rad`'s order (see
            `invkin.neuromechfly.BodyPlan.dof_names`).
        actuated_dof_order: `fly.get_actuated_jointdofs_order(actuator_type)`.

    Returns:
        Index array such that `ik_dofangles_rad[..., index]` is reordered to
        `actuated_dof_order`.
    """
    name_to_idx = {name: i for i, name in enumerate(dof_names)}
    index = [name_to_idx[dof.name] for dof in actuated_dof_order]
    return np.array(index, dtype=np.int64)


def set_tarsal_joint_stiffness(
    world: FlatGroundWorld, fly: NeuroMechFly, stiffness: float
) -> None:
    """Override the passive spring-damper stiffness of every tarsal joint.

    The tarsal segments (tarsus1-2, 2-3, 3-4, 4-5) carry no actuator of
    their own -- unlike every other leg joint, their behavior is governed
    entirely by this passive stiffness (plus damping, left at
    `fly.add_joints`'s own default). Must be called after `world.add_fly`,
    since the fly's joints are namespaced under its own name only once
    attached.
    """
    prefix = f"{fly.name}/"
    for joint_element in world.mjcf_root.worldbody.find_all("joint"):
        if not joint_element.name.startswith(prefix):
            continue  # e.g. the free joint that attaches the fly to the world
        dof = JointDOF.from_name(joint_element.name.removeprefix(prefix))
        if dof.child.link.startswith("tarsus") and dof.child.link != "tarsus1":
            # As of MuJoCo 3.7, a joint's stiffness/damping are stored as
            # polynomial coefficients; index 0 is the linear term (the
            # scalar we want to change).
            joint_element.stiffness[0] = stiffness


def compute_lookat_xyaxes(
    camera_pos: tuple[float, float, float],
    target: tuple[float, float, float] = (0.0, 0.0, 0.0),
    world_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[float, float, float, float, float, float]:
    """MJCF `xyaxes` for a camera at `camera_pos` looking at `target`.

    A MuJoCo camera looks along its own local -Z axis with +Y as image "up";
    `xyaxes` gives the camera's local X and Y axes (in its parent's frame),
    with Z implied as their cross product.
    """
    camera_pos = np.asarray(camera_pos, dtype=float)
    target = np.asarray(target, dtype=float)
    world_up = np.asarray(world_up, dtype=float)
    forward = target - camera_pos
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    return tuple(right) + tuple(true_up)


def quat_yaw(quat: np.ndarray) -> float:
    """Yaw angle (radians, about world Z) encoded in a (w, x, y, z) quaternion."""
    w, x, y, z = quat
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def compute_camera_pose(
    fly_xy: np.ndarray, azimuth_rad: float
) -> tuple[np.ndarray, np.ndarray]:
    """World position and rotation matrix for the orbiting camera.

    Computed explicitly every frame and written directly into
    `mj_data.cam_xpos`/`cam_xmat` (see `replay_period`), rather than
    derived from MuJoCo's own body-kinematics machinery. This keeps camera
    height, elevation, and roll (about its own viewing axis) exactly fixed
    -- only azimuth (position around the fly, about world Z) varies.

    Args:
        fly_xy: `(2,)`, the fly's current thorax (x, y) position, mm.
        azimuth_rad: Camera's angular position around the fly, about world Z.

    Returns:
        camera_pos: `(3,)` world position, mm.
        cam_xmat: `(9,)`, row-major flattened world rotation matrix.
    """
    elevation_rad = np.radians(ORBIT_CAM_ELEVATION_DEG)
    horizontal_distance_mm = ORBIT_CAM_DISTANCE_MM * np.cos(elevation_rad)
    height_above_target_mm = ORBIT_CAM_DISTANCE_MM * np.sin(elevation_rad)
    # Fixed nominal thorax height, not the fly's own (possibly bobbing or
    # falling) current z: both the camera's z and its look-at target's z
    # are constants, which is what keeps the elevation angle itself fixed.
    target_z = SPAWN_POSITION[2]

    direction = np.array([np.cos(azimuth_rad), np.sin(azimuth_rad)])
    camera_xy = fly_xy - horizontal_distance_mm * direction
    camera_pos = np.array(
        [camera_xy[0], camera_xy[1], target_z + height_above_target_mm]
    )
    target_pos = np.array([fly_xy[0], fly_xy[1], target_z])

    xyaxes = compute_lookat_xyaxes(camera_pos, target_pos)
    right, true_up = np.array(xyaxes[:3]), np.array(xyaxes[3:])
    z_axis = np.cross(right, true_up)
    cam_xmat = np.column_stack([right, true_up, z_axis]).flatten()
    return camera_pos, cam_xmat


def build_fly_and_world(
    actuator_gain: float = DEFAULT_ACTUATOR_GAIN,
    adhesion_gain: float = DEFAULT_ADHESION_GAIN,
    tarsal_stiffness: float = DEFAULT_TARSAL_STIFFNESS,
) -> tuple[NeuroMechFly, FlatGroundWorld, mj.MjsCamera]:
    """Build the leg-actuated NeuroMechFly model, world, and orbiting camera.

    Mirrors `flygym/tutorials/2_replaying_experimental_recordings.ipynb`:
    legs-only skeleton, position actuators on the active leg DOFs, leg
    adhesion. `world.add_fly`'s own default already adds ground-contact
    sensors.

    Returns:
        fly: The configured fly.
        world: A `FlatGroundWorld` with `fly` spawned on it.
        orbit_cam: Camera element, for `Simulation.set_renderer`. Its
            compiled pos/quat are placeholders, overwritten every frame
            (see `compute_camera_pose`/`replay_period`); only `fovy` is
            actually used from the compiled spec.
    """
    fly = NeuroMechFly()
    skeleton = Skeleton(
        axis_order=AxisOrder.YAW_PITCH_ROLL, joint_preset=JointPreset.LEGS_ONLY
    )
    fly.add_joints(skeleton, neutral_pose=KinematicPosePreset.NEUTRAL)

    actuated_dofs = fly.skeleton.get_actuated_dofs_from_preset(
        ActuatedDOFPreset.LEGS_ACTIVE_ONLY
    )
    fly.add_actuators(
        actuated_dofs,
        actuator_type=ActuatorType.POSITION,
        kp=actuator_gain,
        neutral_input=KinematicPosePreset.NEUTRAL,
    )
    fly.add_leg_adhesion(gain=adhesion_gain)
    fly.colorize()

    world = FlatGroundWorld()
    half_size_mm = world.ground_geom.size[0]
    texrepeat = half_size_mm / GROUND_CHECKER_SQUARE_MM
    world.mjcf_root.material("grid").texrepeat = [texrepeat, texrepeat]
    world.add_fly(fly, SPAWN_POSITION, SPAWN_ROTATION)
    set_tarsal_joint_stiffness(world, fly, tarsal_stiffness)
    orbit_cam = world.mjcf_root.worldbody.add_camera(
        name="orbitcam", pos=(0.0, 0.0, 1.0), fovy=ORBIT_CAM_FOVY_DEG
    )
    return fly, world, orbit_cam


def restart_physics(sim: Simulation, fly_name: str, neutral_keyframe_id: int) -> None:
    """Reset physics state (pose, velocity, adhesion) to the spawn pose.

    Does NOT reset the renderer (unlike `Simulation.reset()`, which would
    also wipe its already-buffered frames) or the control loop's own clock:
    `mj_resetDataKeyframe` also resets `mj_data.time` to the keyframe's own
    stored time, which would desync the zero-order-hold control loop in
    `replay_period` if left alone, so it's restored right after.
    """
    elapsed_time = sim.mj_data.time
    mj.mj_resetDataKeyframe(sim.mj_model, sim.mj_data, neutral_keyframe_id)
    sim.mj_data.time = elapsed_time
    sim.set_leg_adhesion_states(fly_name, np.ones(6, dtype=bool))
    # mj_resetDataKeyframe doesn't itself propagate the restored qpos into
    # xpos/xquat (needed immediately below, to recompute the camera's pose).
    mj.mj_kinematics(sim.mj_model, sim.mj_data)


def _step_control_frames(
    sim: Simulation,
    fly_name: str,
    thorax_idx: int,
    target_angles: np.ndarray,
    control_freq_hz: float,
    fall_up_z_threshold: float,
    neutral_keyframe_id: int,
    max_restarts: int,
):
    """Shared physics-stepping core for `replay_period` (rendered) and
    `record_period_yaw` (a cheap, renderer-less dry run of the same
    physics, for smoothing the replay camera's heading beforehand -- see
    `replay_period`'s own `camera_azimuth_rad`). Both need bit-identical
    physics (same target angles, same restart-on-fall behavior, no RNG
    anywhere in this model), so this is the one place that logic lives.

    Advances `sim` one control step at a time (zero-order-held actuator
    targets, restarting on a fall as needed -- see `restart_physics`),
    yielding `(thorax_quat, body_pos)` after each completed step.
    """
    control_dt = 1.0 / control_freq_hz
    start_time = sim.time
    # Half a physics step of slack absorbs floating-point rounding in the
    # time comparison, without letting a whole extra physics step slip in.
    half_physics_step = 0.5 * PHYSICS_TIMESTEP

    n_restarts = 0
    warned_restart_cap = False

    for step_idx in range(target_angles.shape[0]):
        sim.set_actuator_inputs(
            fly_name, ActuatorType.POSITION, target_angles[step_idx]
        )
        target_time = start_time + (step_idx + 1) * control_dt
        while sim.time < target_time - half_physics_step:
            sim.step()

            body_quat = sim.get_body_rotations(fly_name)
            thorax_quat = body_quat[thorax_idx]
            up_z = 1.0 - 2.0 * (thorax_quat[1] ** 2 + thorax_quat[2] ** 2)
            if up_z < fall_up_z_threshold:
                if n_restarts < max_restarts:
                    restart_physics(sim, fly_name, neutral_keyframe_id)
                    n_restarts += 1
                    continue
                elif not warned_restart_cap:
                    logger.warning(
                        f"Fly kept tipping over after {max_restarts} restarts in "
                        "this period; no longer restarting for its remainder."
                    )
                    warned_restart_cap = True

        yield thorax_quat, sim.get_body_positions(fly_name)


def record_period_yaw(
    fly: NeuroMechFly,
    world: FlatGroundWorld,
    target_angles: np.ndarray,
    control_freq_hz: float,
    fall_tilt_threshold_deg: float = DEFAULT_FALL_TILT_THRESHOLD_DEG,
    max_restarts: int = DEFAULT_MAX_RESTARTS_PER_PERIOD,
) -> np.ndarray:
    """Physics-only dry run of one period, no renderer attached: the
    thorax's own per-step yaw (radians, about world Z, before
    `ORBIT_CAM_AZIMUTH_OFFSET_DEG`), for smoothing the replay camera's
    heading before the real (rendered) pass -- see `replay_period`'s own
    `camera_azimuth_rad`. Steps the exact same deterministic physics
    `replay_period` will step again for the real pass (see
    `_step_control_frames`), just without a renderer, so it's cheap
    relative to it.

    Args: see `replay_period`.
    """
    fly_name = fly.name
    sim = Simulation(world, timestep=PHYSICS_TIMESTEP)
    neutral_keyframe_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_KEY, "neutral")
    thorax_idx = fly.get_bodysegs_order().index(THORAX_SEGMENT)
    fall_up_z_threshold = np.cos(np.radians(fall_tilt_threshold_deg))

    sim.reset()
    sim.set_leg_adhesion_states(fly_name, np.ones(6, dtype=bool))
    sim.warmup()

    steps = _step_control_frames(
        sim, fly_name, thorax_idx, target_angles, control_freq_hz,
        fall_up_z_threshold, neutral_keyframe_id, max_restarts,
    )  # fmt: skip
    return np.array([quat_yaw(thorax_quat) for thorax_quat, _ in steps])


def replay_period(
    fly: NeuroMechFly,
    world: FlatGroundWorld,
    orbit_cam: mj.MjsCamera,
    target_angles: np.ndarray,
    control_freq_hz: float,
    panel_size: int,
    fall_tilt_threshold_deg: float = DEFAULT_FALL_TILT_THRESHOLD_DEG,
    max_restarts: int = DEFAULT_MAX_RESTARTS_PER_PERIOD,
    camera_azimuth_rad: np.ndarray | None = None,
) -> np.ndarray:
    """Replay one period and return its rendered frames.

    Args:
        fly, world, orbit_cam: See `build_fly_and_world`.
        target_angles: `(n_steps, n_actuated_dofs)` position-actuator
            targets, in `fly.get_actuated_jointdofs_order(POSITION)` order,
            one recorded frame per control update.
        control_freq_hz: Actuator targets update at this rate (zero-order
            hold), independent of `PHYSICS_TIMESTEP`.
        panel_size: Output height and width (square) in pixels.
        fall_tilt_threshold_deg: Restart (see `restart_physics`) once the
            thorax's own up-vector tips this many degrees from vertical.
        max_restarts: Give up restarting (but keep simulating and
            rendering) after this many restarts in this period, so a
            period whose IK is too poor to ever recover doesn't loop
            forever.
        camera_azimuth_rad: `(n_steps,)`, already including
            `ORBIT_CAM_AZIMUTH_OFFSET_DEG`. If given, used directly as
            this period's camera azimuth each step, instead of tracking
            the thorax's own live (unsmoothed) yaw -- see
            `record_period_yaw` to compute a time-smoothed version of it
            first. `None` (default) tracks yaw live, unsmoothed.

    Returns:
        `(n_steps, panel_size, panel_size, 3)` uint8 RGB frames, one per
        `target_angles` row.
    """
    fly_name = fly.name
    sim = Simulation(world, timestep=PHYSICS_TIMESTEP)
    sim.set_renderer(
        orbit_cam,
        camera_res=(panel_size, panel_size),
        playback_speed=1.0,
        output_fps=control_freq_hz,
    )

    neutral_keyframe_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_KEY, "neutral")
    cam_id = mj.mj_name2id(sim.mj_model, mj.mjtObj.mjOBJ_CAMERA, orbit_cam.name)
    thorax_idx = fly.get_bodysegs_order().index(THORAX_SEGMENT)
    fall_up_z_threshold = np.cos(np.radians(fall_tilt_threshold_deg))

    sim.reset()
    sim.set_leg_adhesion_states(fly_name, np.ones(6, dtype=bool))
    sim.warmup()

    steps = _step_control_frames(
        sim, fly_name, thorax_idx, target_angles, control_freq_hz,
        fall_up_z_threshold, neutral_keyframe_id, max_restarts,
    )  # fmt: skip
    for step_idx, (thorax_quat, body_pos) in enumerate(steps):
        # Render exactly once per control step, using this step's final
        # pose. `render_as_needed`'s own fps-based timing gate (meant for
        # playback-speed-scaled video, not a strict one-frame-per-input-row
        # mapping) can silently fire a different number of times than
        # `target_angles.shape[0]` whenever `control_freq_hz` doesn't evenly
        # divide `PHYSICS_TIMESTEP` (e.g. 396 Hz doesn't divide 1e4 Hz), so
        # force it here instead of trusting that gate.
        if camera_azimuth_rad is None:
            azimuth_rad = quat_yaw(thorax_quat) + np.radians(
                ORBIT_CAM_AZIMUTH_OFFSET_DEG
            )
        else:
            azimuth_rad = camera_azimuth_rad[step_idx]
        camera_pos, cam_xmat = compute_camera_pose(
            body_pos[thorax_idx, :2], azimuth_rad
        )
        sim.mj_data.cam_xpos[cam_id] = camera_pos
        sim.mj_data.cam_xmat[cam_id] = cam_xmat
        sim.renderer._last_render_time_sec = -np.inf
        sim.render_as_needed()

    rendered = sim.renderer.frames[orbit_cam.name]
    assert len(rendered) == target_angles.shape[0]
    return np.stack(rendered)


# One (fly, world, orbit_cam) per worker PROCESS, not per period: built
# lazily on first use and reused for every subsequent period a persistent
# joblib/loky worker picks up (see `replay_period_in_worker`). NeuroMechFly/
# FlatGroundWorld/MjsCamera aren't picklable, so they can't be built once in
# the caller and passed in -- and rebuilding the whole body plan per period
# (rather than just recompiling the per-period `Simulation`, which
# `replay_period` already does) would otherwise dominate the wall-clock cost
# of replaying hundreds of short periods.
_worker_fly_world: tuple[NeuroMechFly, FlatGroundWorld, mj.MjsCamera] | None = None


def replay_period_in_worker(
    target_angles: np.ndarray,
    control_freq_hz: float,
    panel_size: int,
    actuator_gain: float = DEFAULT_ACTUATOR_GAIN,
    adhesion_gain: float = DEFAULT_ADHESION_GAIN,
    tarsal_stiffness: float = DEFAULT_TARSAL_STIFFNESS,
    **replay_period_kwargs,
) -> np.ndarray:
    """`replay_period`, for use as a `joblib.delayed` target across periods.

    See module-level `_worker_fly_world` for why the fly/world are built
    here (once per worker process) rather than passed in. The
    actuator/adhesion/tarsal-stiffness knobs only take effect on that first
    build; every call in one `replay_physics.py` invocation passes the same
    values anyway (one CLI call, one set of physics knobs), so the cache
    never goes stale within a run.
    """
    global _worker_fly_world
    if _worker_fly_world is None:
        _worker_fly_world = build_fly_and_world(
            actuator_gain=actuator_gain,
            adhesion_gain=adhesion_gain,
            tarsal_stiffness=tarsal_stiffness,
        )
    fly, world, orbit_cam = _worker_fly_world
    return replay_period(
        fly, world, orbit_cam, target_angles, control_freq_hz, panel_size,
        **replay_period_kwargs,
    )  # fmt: skip


def record_period_yaw_in_worker(
    target_angles: np.ndarray,
    control_freq_hz: float,
    actuator_gain: float = DEFAULT_ACTUATOR_GAIN,
    adhesion_gain: float = DEFAULT_ADHESION_GAIN,
    tarsal_stiffness: float = DEFAULT_TARSAL_STIFFNESS,
    **record_period_yaw_kwargs,
) -> np.ndarray:
    """`record_period_yaw`, for use as a `joblib.delayed` target across
    periods -- see `replay_period_in_worker`'s own docstring for why the
    fly/world are built here (once per worker process) rather than passed
    in; same per-worker cache, shared with `replay_period_in_worker`.
    """
    global _worker_fly_world
    if _worker_fly_world is None:
        _worker_fly_world = build_fly_and_world(
            actuator_gain=actuator_gain,
            adhesion_gain=adhesion_gain,
            tarsal_stiffness=tarsal_stiffness,
        )
    fly, world, _ = _worker_fly_world
    return record_period_yaw(
        fly, world, target_angles, control_freq_hz, **record_period_yaw_kwargs
    )
