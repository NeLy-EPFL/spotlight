"""I/O for the postprocessing pipeline's dense per-frame `kinematics.h5`:
one row per frame of the *whole* trial, no period/segment concept (curation
of "which stretches are good enough for e.g. FlyGym replay" now happens
downstream, in poseforge2, using this file's own per-frame confidence/
mismatch data -- see `scripts/postprocessing/solve_ik.py`'s module docstring).

Schema
------
pose2d/ (always present)
    keypoint_positions_2d_px    (n_frames, n_keypoints, 2), raw prediction,
                                 aligned pixel domain
    keypoint_positions_2d_mm    (n_frames, n_keypoints, 2), raw prediction,
                                 physical mm
    keypoint_positions_confidence  (n_frames, n_keypoints)
    attrs: keypoint_order (list[str])
inverse_kinematics/ (present only if IK was run)
    dof_angles                   (n_frames, n_dofs), radians, NaN for any
                                  frame IK wasn't attempted for
    keypoint_positions_3d_mm     (n_frames, n_keypoints, 3), FK at the
                                  converged pose, NaN likewise
    keypoint_positions_2d_px     (n_frames, n_keypoints, 2), the above
                                  mapped back to the aligned pixel domain
    attrs: keypoint_order (list[str], same order as pose2d/), dof_names
        (list[str]), neutral_weight (float), keypoint_weight_scale (a
        keypoint-name -> weight JSON dict, h5py attrs don't hold a real
        dict directly)
"""

import json
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np


@dataclass
class InverseKinematicsResult:
    """`inverse_kinematics/`'s own datasets/attrs -- see this module's
    docstring for field-by-field shapes."""

    dof_names: list[str]
    dof_angles: np.ndarray
    keypoint_positions_3d_mm: np.ndarray
    keypoint_positions_2d_px: np.ndarray
    neutral_weight: float
    keypoint_weight_scale: dict[str, float]


def save_kinematics_h5(
    output_path: Path,
    *,
    node_names: list[str],
    keypoint_positions_2d_px: np.ndarray,
    keypoint_positions_2d_mm: np.ndarray,
    keypoint_positions_confidence: np.ndarray,
    inverse_kinematics: InverseKinematicsResult | None,
) -> None:
    """Save one trial's dense per-frame pose2d (+ optional IK) results.

    Args:
        output_path: Where to save the `.h5` file.
        node_names: Keypoint order, matching every array's keypoint axis.
        keypoint_positions_2d_px: `(n_frames, n_keypoints, 2)`.
        keypoint_positions_2d_mm: `(n_frames, n_keypoints, 2)`.
        keypoint_positions_confidence: `(n_frames, n_keypoints)`.
        inverse_kinematics: The IK fit, or `None` to write a pose2d-only
            file (i.e. IK wasn't requested at all for this trial).
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        pose2d = f.create_group("pose2d")
        pose2d.attrs["keypoint_order"] = node_names
        pose2d.create_dataset(
            "keypoint_positions_2d_px", data=keypoint_positions_2d_px, compression="gzip"
        )  # fmt: skip
        pose2d.create_dataset(
            "keypoint_positions_2d_mm", data=keypoint_positions_2d_mm, compression="gzip"
        )  # fmt: skip
        pose2d.create_dataset(
            "keypoint_positions_confidence",
            data=keypoint_positions_confidence,
            compression="gzip",
        )

        if inverse_kinematics is not None:
            ik = inverse_kinematics
            group = f.create_group("inverse_kinematics")
            group.attrs["keypoint_order"] = node_names
            group.attrs["dof_names"] = ik.dof_names
            group.attrs["neutral_weight"] = ik.neutral_weight
            group.attrs["keypoint_weight_scale"] = json.dumps(ik.keypoint_weight_scale)
            group.create_dataset("dof_angles", data=ik.dof_angles, compression="gzip")
            group.create_dataset(
                "keypoint_positions_3d_mm",
                data=ik.keypoint_positions_3d_mm,
                compression="gzip",
            )
            group.create_dataset(
                "keypoint_positions_2d_px",
                data=ik.keypoint_positions_2d_px,
                compression="gzip",
            )


def load_kinematics_h5(input_path: Path) -> dict:
    """Load a `.h5` file written by `save_kinematics_h5` back into a plain dict.

    Args:
        input_path: `.h5` file to load.

    Returns:
        Dict with keys `node_names`, `keypoint_positions_2d_px`,
        `keypoint_positions_2d_mm`, `keypoint_positions_confidence`, and
        `inverse_kinematics` (an `InverseKinematicsResult`, or `None` if
        the file has no `inverse_kinematics/` group).
    """
    with h5py.File(input_path, "r") as f:
        pose2d = f["pose2d"]
        result = {
            "node_names": [str(n) for n in pose2d.attrs["keypoint_order"]],
            "keypoint_positions_2d_px": pose2d["keypoint_positions_2d_px"][:],
            "keypoint_positions_2d_mm": pose2d["keypoint_positions_2d_mm"][:],
            "keypoint_positions_confidence": pose2d["keypoint_positions_confidence"][:],
            "inverse_kinematics": None,
        }
        if "inverse_kinematics" in f:
            group = f["inverse_kinematics"]
            result["inverse_kinematics"] = InverseKinematicsResult(
                dof_names=[str(n) for n in group.attrs["dof_names"]],
                dof_angles=group["dof_angles"][:],
                keypoint_positions_3d_mm=group["keypoint_positions_3d_mm"][:],
                keypoint_positions_2d_px=group["keypoint_positions_2d_px"][:],
                neutral_weight=float(group.attrs["neutral_weight"]),
                keypoint_weight_scale=json.loads(group.attrs["keypoint_weight_scale"]),
            )
    return result
