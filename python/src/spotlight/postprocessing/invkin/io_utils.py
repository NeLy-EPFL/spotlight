"""I/O for `inverse_kinematics.h5`: one row per frame of the *whole* trial,
no period/segment concept (curation of "which stretches are good enough for
e.g. FlyGym replay" happens downstream, using this file's own per-frame
confidence/mismatch data: see `invkin.solve_ik`'s module docstring).

This file holds IK-specific results only; the raw 2D pose predictions it was
fit from live separately in `pose2d.h5` (see `pose2d.io_utils.save_pose_h5`).

Schema
------
dof_angles                   (n_frames, n_dofs), radians, NaN for any frame
                              IK wasn't attempted for
keypoint_positions_3d_mm     (n_frames, n_keypoints, 3), FK at the converged
                              pose, NaN likewise
keypoint_positions_2d_px     (n_frames, n_keypoints, 2), the above mapped
                              back to the aligned pixel domain
mismatch_mask                (n_frames,) bool: IK was attempted, its
                              fk-to-prediction mismatch is within
                              `max_mismatch`, and this is already denoised
                              over `mismatch_denoise_window_sec` (see
                              `solve_ik.py`'s `compute_mismatch_mask`): the
                              QA video's own display-time IK-acceptance
                              gate reads this directly rather than
                              recomputing it.
attrs: keypoint_order (list[str]), dof_names (list[str]),
    neutral_weight (float), keypoint_weight_scale (a keypoint-name -> weight
    JSON dict, h5py attrs don't hold a real dict directly), max_mismatch
    (float, mm), mismatch_denoise_window_sec (float)
"""

import json
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np


@dataclass
class InverseKinematicsResult:
    """`inverse_kinematics.h5`'s own datasets/attrs: see this module's
    docstring for field-by-field shapes."""

    dof_names: list[str]
    dof_angles: np.ndarray
    keypoint_positions_3d_mm: np.ndarray
    keypoint_positions_2d_px: np.ndarray
    mismatch_mask: np.ndarray
    neutral_weight: float
    keypoint_weight_scale: dict[str, float]
    max_mismatch: float
    mismatch_denoise_window_sec: float


def save_inverse_kinematics_h5(
    output_path: Path,
    *,
    node_names: list[str],
    result: InverseKinematicsResult,
) -> None:
    """Save one trial's dense per-frame IK fit.

    Args:
        output_path: Where to save the `.h5` file.
        node_names: Keypoint order, matching every array's keypoint axis
            (same order as the `pose2d.h5` this was fit from).
        result: The IK fit.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        f.attrs["keypoint_order"] = node_names
        f.attrs["dof_names"] = result.dof_names
        f.attrs["neutral_weight"] = result.neutral_weight
        f.attrs["keypoint_weight_scale"] = json.dumps(result.keypoint_weight_scale)
        f.attrs["max_mismatch"] = result.max_mismatch
        f.attrs["mismatch_denoise_window_sec"] = result.mismatch_denoise_window_sec
        f.create_dataset("dof_angles", data=result.dof_angles, compression="gzip")
        f.create_dataset(
            "keypoint_positions_3d_mm",
            data=result.keypoint_positions_3d_mm,
            compression="gzip",
        )
        f.create_dataset(
            "keypoint_positions_2d_px",
            data=result.keypoint_positions_2d_px,
            compression="gzip",
        )
        f.create_dataset("mismatch_mask", data=result.mismatch_mask, compression="gzip")


def load_inverse_kinematics_h5(input_path: Path) -> InverseKinematicsResult:
    """Load a `.h5` file written by `save_inverse_kinematics_h5`.

    Args:
        input_path: `.h5` file to load.
    """
    with h5py.File(input_path, "r") as f:
        return InverseKinematicsResult(
            dof_names=[str(n) for n in f.attrs["dof_names"]],
            dof_angles=f["dof_angles"][:],
            keypoint_positions_3d_mm=f["keypoint_positions_3d_mm"][:],
            keypoint_positions_2d_px=f["keypoint_positions_2d_px"][:],
            mismatch_mask=f["mismatch_mask"][:],
            neutral_weight=float(f.attrs["neutral_weight"]),
            keypoint_weight_scale=json.loads(f.attrs["keypoint_weight_scale"]),
            max_mismatch=float(f.attrs["max_mismatch"]),
            mismatch_denoise_window_sec=float(f.attrs["mismatch_denoise_window_sec"]),
        )
