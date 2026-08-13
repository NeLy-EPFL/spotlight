"""I/O for `physics_replay.h5`: FlyGym CPU-physics replay of one trial's
solved IK joint angles (see `replay.flygym`), as a persisted pipeline
stage rather than a per-run throwaway file. `replay.physics.replay_physics`
writes this; `postprocessing.visualize`'s QA video reads it back for the
row-2 replay panel.

Schema
------
frames        (n_rendered, panel_size, panel_size, 3) uint8 RGB: one
              rendered frame per frame `replay_physics` actually attempted
              (every frame in an IK-attempted stretch, see `find_ik_runs`),
              concatenated across stretches in trial order. NOT one row per
              trial frame: unattempted frames have no row at all.
frame_lookup  (n_frames,) int64: index into `frames` for each of the trial's
              real frames, or `-1` wherever IK wasn't attempted (so replay
              wasn't run either). The QA video applies its own display-time
              mismatch/flip gating on top of this (see `visualize.py`),
              the same way it does for `inverse_kinematics.h5`'s own
              `keypoint_positions_3d_mm`.
attrs: actuator_gain (float), adhesion_gain (float), tarsal_stiffness
    (float), fall_tilt_threshold_deg (float)
"""

from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np


@dataclass
class PhysicsReplayResult:
    frames: np.ndarray
    """`(n_rendered, panel_size, panel_size, 3)` uint8 RGB."""
    frame_lookup: np.ndarray
    """`(n_frames,)` int64, `-1` wherever replay wasn't run."""
    actuator_gain: float
    adhesion_gain: float
    tarsal_stiffness: float
    fall_tilt_threshold_deg: float


def save_physics_replay_h5(output_path: Path, result: PhysicsReplayResult) -> None:
    """Save one trial's FlyGym replay. See module docstring for schema.

    Uses the same one-frame-per-HDF5-chunk, gzip-level-1 convention as
    `io.MuscleH5Writer` (profiled there: the dominant cost of a much higher
    default compression level, for little size benefit on this kind of
    per-frame image data).
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    panel_size = result.frames.shape[1]
    with h5py.File(output_path, "w") as f:
        f.create_dataset(
            "frames",
            data=result.frames,
            chunks=(1, panel_size, panel_size, 3),
            compression="gzip",
            compression_opts=1,
        )
        f.create_dataset("frame_lookup", data=result.frame_lookup, compression="gzip")
        f.attrs["actuator_gain"] = result.actuator_gain
        f.attrs["adhesion_gain"] = result.adhesion_gain
        f.attrs["tarsal_stiffness"] = result.tarsal_stiffness
        f.attrs["fall_tilt_threshold_deg"] = result.fall_tilt_threshold_deg


def load_physics_replay_frame_lookup(input_path: Path) -> np.ndarray:
    """Just `frame_lookup` (`(n_frames,)`, cheap), without touching
    `frames` (`(n_rendered, panel_size, panel_size, 3)` uint8, big enough
    that a chunked renderer wants to read only its own chunk's rows lazily
    from the file rather than load the whole thing: see `visualize.py`'s
    `_render_chunk`, which opens `input_path` itself for exactly that)."""
    with h5py.File(input_path, "r") as f:
        return f["frame_lookup"][:]


def load_physics_replay_h5(input_path: Path) -> PhysicsReplayResult:
    """Load a `.h5` file written by `save_physics_replay_h5`."""
    with h5py.File(input_path, "r") as f:
        return PhysicsReplayResult(
            frames=f["frames"][:],
            frame_lookup=f["frame_lookup"][:],
            actuator_gain=float(f.attrs["actuator_gain"]),
            adhesion_gain=float(f.attrs["adhesion_gain"]),
            tarsal_stiffness=float(f.attrs["tarsal_stiffness"]),
            fall_tilt_threshold_deg=float(f.attrs["fall_tilt_threshold_deg"]),
        )
