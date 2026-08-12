"""Shared 2D affine transform helpers for pose coordinate conversions.

Used by `scripts/slp_apply_alignment.py` (raw camera domain <-> aligned
domain) and `scripts/calibration.py` (aligned domain -> raw camera domain,
for the physical mm conversion).
"""

import numpy as np


def apply_affine(points: np.ndarray, matrices: np.ndarray) -> np.ndarray:
    """Apply per-frame 2x3 affine matrices to per-frame keypoints.

    Args:
        points: `(n_frames, n_nodes, 2)` keypoint coordinates, may contain NaN.
        matrices: `(n_frames, 2, 3)` affine transformation matrices, one per frame.

    Returns:
        `(n_frames, n_nodes, 2)` transformed keypoint coordinates.
    """
    ones = np.ones((*points.shape[:2], 1), dtype=points.dtype)
    points_h = np.concatenate([points, ones], axis=-1)  # (n_frames, n_nodes, 3)
    return np.einsum("fij,fnj->fni", matrices, points_h)


def invert_affine(matrices: np.ndarray) -> np.ndarray:
    """Invert per-frame 2x3 affine matrices.

    Args:
        matrices: `(n_frames, 2, 3)` affine transformation matrices.

    Returns:
        `(n_frames, 2, 3)` affine matrices, each the inverse of the input.
    """
    n_frames = matrices.shape[0]
    bottom_row = np.tile(np.array([[[0.0, 0.0, 1.0]]]), (n_frames, 1, 1))
    square = np.concatenate([matrices, bottom_row], axis=1)  # (n_frames, 3, 3)
    return np.linalg.inv(square)[:, :2, :]
