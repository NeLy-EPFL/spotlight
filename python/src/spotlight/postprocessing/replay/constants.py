"""Tunable FlyGym-replay constants. Plain numbers only, no flygym/mujoco
import, so the CLI can read them for its own default values without
requiring flygym installed just to parse `--help` or run with
`--physics-replay.enabled` off. See `replay.flygym` for what each one
actually does.
"""

# FlyGym's own default (see mujoco_globals.yaml). A much coarser physics
# step makes the contact-rich leg/ground dynamics numerically unstable.
PHYSICS_TIMESTEP = 1e-4  # seconds

# Validated against this same QuickIK-solved body plan.
DEFAULT_ACTUATOR_GAIN = 75.0  # uN*mm/rad (FlyGym's own "kp" position-actuator gain)
DEFAULT_ADHESION_GAIN = 12.0  # uN
DEFAULT_TARSAL_STIFFNESS = 8.0

# Restart-on-fall: tip past this many degrees from vertical (measured from
# the thorax's own up-vector) and the physics state is reset.
DEFAULT_FALL_TILT_THRESHOLD_DEG = 60.0
DEFAULT_MAX_RESTARTS_PER_PERIOD = 5

# Orbiting camera: same elevation as the QA video's own synthetic-3D panel.
# Distance and azimuth offset picked by rendering a sweep and comparing by eye.
ORBIT_CAM_DISTANCE_MM = 12.0
ORBIT_CAM_ELEVATION_DEG = 30.0
ORBIT_CAM_FOVY_DEG = 30.0
ORBIT_CAM_AZIMUTH_OFFSET_DEG = 220.0

# Ground checker floor: `FlatGroundWorld`'s own default (4 mm/square) is
# coarser than the IK 3D panel's own 1 mm grid; matched here so the two
# side-by-side panels look consistent.
GROUND_CHECKER_SQUARE_MM = 1.0
