"""Shared logic for finding contiguous "good" periods in a per-frame mask.

Used by `tools/spotlight_ik/solve_ik.py`, for the initial confidence-based
period split, the IK/FK-mismatch-based refinement, and the joint-movement-
based refinement (see `compute_joint_excursion_deg`).
"""

import numpy as np
from scipy import ndimage


def leg_for_dof_name(dof_name: str) -> str:
    """The leg prefix (`lf`/`lm`/`lh`/`rf`/`rm`/`rh`) a DOF belongs to.

    DOF names are FlyGym's own `"{parent}-{child}-{axis}"` convention (see
    `spotlight_ik.flygym_replay.build_dof_reorder_index`); the child link's
    own name always starts with its leg's prefix (e.g. `"c_thorax-lf_coxa-
    yaw"`'s child is `"lf_coxa"`), even for the thorax-coxa DOFs, whose
    parent is the root (`"c_thorax"`) rather than another leg link.

    Args:
        dof_name: One `"{parent}-{child}-{axis}"` DOF name.

    Returns:
        The two-letter leg prefix.
    """
    return dof_name.split("-")[1].split("_")[0]


def compute_joint_excursion_deg(
    dof_angles_rad: np.ndarray, dof_names: list[str], window_frames: int
) -> np.ndarray:
    """Per-frame joint excursion, in degrees: how much any single leg's own
    DOFs sweep through within a window centered on each frame.

    A windowed range (max - min), not a frame-to-frame derivative:
    differentiating would amplify whatever per-frame IK noise survives
    median filtering (see `solve_ik.py`'s own filtering step) into a false
    "moving" signal, since differentiation is a high-pass operation. A
    windowed range doesn't have that problem, since noise that doesn't
    accumulate directionally within the window contributes comparatively
    little to it, unlike a genuine, sustained joint movement (confirmed
    empirically: real held-still moments in this project's own data read a
    few degrees at a 50ms window, genuine movement tens of degrees).

    Args:
        dof_angles_rad: `(period_length, n_dofs)`, radians (should already
            be median-filtered; see `solve_ik.py`'s own filtering step,
            applied before this).
        dof_names: DOF names matching `dof_angles_rad`'s last axis.
        window_frames: Sliding window size, in frames. Should be long
            enough to not be swamped by residual per-frame noise, but short
            enough to still resolve genuinely brief movements (e.g. a quick
            grooming stroke) -- on the order of 100ms empirically.

    Returns:
        `(period_length,)`, degrees: for each frame, the largest single-DOF
        range, among any one leg's own 7 DOFs, within the window centered
        on it. Taken as a max both within a leg's own DOFs and across legs
        (rather than e.g. an L2 norm mixing different joints' own axes),
        so that a single leg moving (even if every other leg is planted)
        is enough to register.
    """
    leg_groups: dict[str, list[int]] = {}
    for i, name in enumerate(dof_names):
        leg_groups.setdefault(leg_for_dof_name(name), []).append(i)

    dof_angles_deg = np.degrees(dof_angles_rad)
    per_leg_excursion = []
    for idxs in leg_groups.values():
        leg_angles = dof_angles_deg[:, idxs]  # (period_length, 7)
        window_max = ndimage.maximum_filter1d(leg_angles, window_frames, axis=0)
        window_min = ndimage.minimum_filter1d(leg_angles, window_frames, axis=0)
        per_leg_excursion.append((window_max - window_min).max(axis=-1))
    return np.stack(per_leg_excursion, axis=-1).max(axis=-1)


def find_periods(
    accepted: np.ndarray, closing_size: int, min_period_length: int
) -> list[tuple[int, int]]:
    """Find contiguous accepted periods in one video, after morphological closing.

    Args:
        accepted: `(n_frames,)` bool, whether each frame was accepted.
        closing_size: Length of the structuring element used for binary
            closing; bridges gaps of up to about this many consecutive
            not-accepted frames.
        min_period_length: Minimum number of frames for a period to be kept.

    Returns:
        List of `(start, end)` frame index pairs (`end` exclusive), one per
        kept period, in chronological order.
    """
    closed = ndimage.binary_closing(
        accepted, structure=np.ones(closing_size, dtype=bool)
    )
    labeled_periods, n_periods = ndimage.label(closed)

    periods = []
    for period_id in range(1, n_periods + 1):
        frame_idxs = np.flatnonzero(labeled_periods == period_id)
        start, end = int(frame_idxs[0]), int(frame_idxs[-1]) + 1
        if end - start >= min_period_length:
            periods.append((start, end))
    return periods
