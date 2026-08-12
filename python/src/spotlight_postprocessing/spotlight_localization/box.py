"""Reconstructs the RepVGG-A0 pipeline's 900x900 aligned-domain bounding
box directly on the raw fullsize frame, from `TinyLocalizationModel`'s own
neck/thorax/abdomen predictions (in raw pixel space) -- for QA
visualization (see `scripts/spotlight_localization/visualize_predictions.py`)
without needing that trial's own real alignment transform.

The "canonical" target this reconstruction is fit against is the AVERAGE
neck/thorax/abdomen position across the entire dataset's own real
aligned-domain predictions (`spotlight_postprocessing.spotlight_pose2d`),
i.e. wherever the real alignment pipeline actually tends to place them,
not a hand-picked convention.

RAW_TO_ALIGNED_SCALE is a fixed constant (1.0), not something fit from
data: `behavior_alignment_transforms.h5`'s own real per-frame transforms
were checked directly (`sqrt(abs(det(linear_part)))` over 200 frames each,
3 different trials/genotypes) and every single one comes out to exactly
1.0 with zero variance -- the real pipeline crops/rotates but never
zooms, so one raw pixel is always exactly one aligned-domain pixel. An
earlier version of this module instead *fit* the scale (first per frame,
then, after that visibly made the box jitter, as the median across many
frames) from `TinyLocalizationModel`'s own predicted points vs. the canonical
target -- both versions quietly baked the model's own keypoint accuracy
into what should be a fixed physical constant, backing out an apparent
"scale" of 1.4-1.6 instead of 1.0. In other words, the model's predicted
neck/thorax/abdomen spread is only 60-70% of the real physical spread --
a real prediction-accuracy problem, not a scale-calibration one.
"""

from pathlib import Path

import numpy as np

from spotlight_postprocessing.spotlight_localization.dataset import (
    coarse_points_aligned_domain,
)
from spotlight_postprocessing.spotlight_pose2d.geometry import (
    apply_affine,
    invert_affine,
)
from spotlight_postprocessing.spotlight_pose2d.io_utils import load_pose_h5

ALIGNED_BOX_SIZE = 900  # spotlight_pose2d.dataset.ALIGNED_FRAME_SIZE
RAW_TO_ALIGNED_SCALE = 1.0  # see module docstring -- a measured physical constant


def measure_canonical_aligned_points(h5_paths: list[Path]) -> np.ndarray:
    """Mean neck/thorax/abdomen position (see `dataset.KEYPOINT_SPEC`), in
    the 900x900 aligned domain, pooled over every frame of every trial in
    `h5_paths` -- the real alignment pipeline's own long-run target
    position/orientation for the fly.

    A handful of frames per trial (a few out of ~20k) have NaN poses even in
    an exhaustively-predicted `final_predictions.h5`; `np.nanmean` skips
    them rather than propagating NaN into the whole average.

    Args:
        h5_paths: Trials' `final_predictions.h5` files.

    Returns:
        `(3, 2)`: mean point per keypoint.
    """
    all_points = []
    for h5_path in h5_paths:
        data = load_pose_h5(h5_path)
        all_points.append(
            coarse_points_aligned_domain(data["poses"], data["node_names"])
        )
    return np.nanmean(np.concatenate(all_points), axis=0)


def fit_line_direction(points: np.ndarray) -> np.ndarray:
    """Best-fit line direction through `points`, via total least squares
    (the first principal component of the centered points, from an SVD) --
    every point's own deviation from the centroid contributes, unlike
    e.g. just connecting two of them and ignoring the third.

    A line has no intrinsic "forward," so the returned direction's sign
    is arbitrary here; `fit_rotation_translation` disambiguates it using
    the points' own known order (first to last) before using it as a
    rotation reference.

    Args:
        points: `(n, 2)`, `n >= 2`.

    Returns:
        `(2,)` unit vector along the fitted line.
    """
    centered = points - points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered)
    return vt[0]


def fit_disambiguated_direction(points: np.ndarray) -> np.ndarray:
    """`fit_line_direction`, sign-resolved using `points`' own first-to-last
    order (e.g. head/neck end to abdomen end) -- see `fit_rotation_
    translation`'s own docstring for why this, not a general shape fit,
    is the right way to read a near-collinear keypoint set's orientation.

    Args:
        points: `(n, 2)`, `n >= 2`, in head-to-abdomen (or equivalent)
            order.

    Returns:
        `(2,)` unit vector, pointing from `points[0]` toward `points[-1]`.
    """
    direction = fit_line_direction(points)
    if np.dot(direction, points[-1] - points[0]) < 0:
        direction = -direction
    return direction


def fit_rotation_translation(
    predicted_points: np.ndarray,
    canonical_points: np.ndarray,
    scale: float = RAW_TO_ALIGNED_SCALE,
    pred_direction_override: np.ndarray | None = None,
) -> np.ndarray:
    """Fits a raw-to-aligned affine transform with a FIXED scale -- only
    rotation and translation vary per frame.

    Rotation comes from the fly's own body axis, not a full Kabsch/
    Procrustes shape fit: `predicted_points`/`canonical_points`'s coarse
    keypoints (e.g. neck/thorax/abdomen) sit close to collinear along the
    body, so `fit_line_direction`'s total-least-squares line -- using all
    `n` points' own deviation from their centroid, not just two of them --
    is a more direct, more noise-robust estimate of the body's own
    orientation than fitting a general 2D shape-matching rotation to a
    near-degenerate (nearly 1D) point configuration would be. The line's
    own 180-degree sign ambiguity is resolved by each point set's
    first-to-last order (e.g. head/neck end to abdomen end), which is
    also what actually distinguishes "rotated 180 degrees" from "not," a
    real orientation this reconstruction needs to get right.

    Args:
        predicted_points: `(n, 2)` coarse keypoints (e.g. neck/thorax/
            abdomen, in that head-to-abdomen order), raw pixel domain.
        canonical_points: `(n, 2)`, same keypoints/order, 900x900 aligned
            domain.
        scale: Raw-to-aligned scale; `RAW_TO_ALIGNED_SCALE` (1.0) unless
            overridden for e.g. a display already resized by some factor.
        pred_direction_override: `(2,)` unit vector to use instead of
            fitting one fresh from `predicted_points` -- e.g. a version of
            this same direction smoothed over time, for a QA overlay that
            shouldn't jitter frame to frame the way an independent per-frame
            fit does. Still sign-disambiguated the same way a freshly-fit
            direction would be. `None` (default) fits fresh, as before.

    Returns:
        `(2, 3)` affine matrix, raw -> aligned domain.
    """
    pred_centroid = predicted_points.mean(axis=0)
    canon_centroid = canonical_points.mean(axis=0)

    if pred_direction_override is not None:
        pred_direction = pred_direction_override
        if np.dot(pred_direction, predicted_points[-1] - predicted_points[0]) < 0:
            pred_direction = -pred_direction
    else:
        pred_direction = fit_disambiguated_direction(predicted_points)
    canon_direction = fit_disambiguated_direction(canonical_points)

    angle = np.arctan2(canon_direction[1], canon_direction[0]) - np.arctan2(
        pred_direction[1], pred_direction[0]
    )
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    rotation = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    linear = scale * rotation
    translation = canon_centroid - linear @ pred_centroid
    return np.hstack([linear, translation.reshape(2, 1)])


def raw_domain_box_corners(
    predicted_points: np.ndarray,
    canonical_points: np.ndarray,
    scale: float = RAW_TO_ALIGNED_SCALE,
    box_size: int = ALIGNED_BOX_SIZE,
    pred_direction_override: np.ndarray | None = None,
) -> np.ndarray | None:
    """Where the 900x900 aligned box's corners would land in the raw frame,
    if `predicted_points` (this frame's own neck/thorax/abdomen, in raw
    pixel space) were aligned the same way `canonical_points` (the
    dataset's own long-run average, in the 900x900 aligned domain) is.

    Fits rotation + translation per frame at a FIXED scale (see
    `fit_rotation_translation`), then inverts the resulting transform to
    map the aligned box's own corners back into raw pixel space -- so the
    reconstructed box is always exactly `box_size` raw pixels on a side
    (900 at `scale=1.0`), matching the real alignment pipeline's own crop.

    Args:
        predicted_points: `(3, 2)` neck/thorax/abdomen, raw pixel domain.
        canonical_points: `(3, 2)` neck/thorax/abdomen, 900x900 aligned
            domain (see `measure_canonical_aligned_points`).
        scale: Raw-to-aligned scale; `RAW_TO_ALIGNED_SCALE` (1.0) unless
            overridden.
        box_size: Aligned domain's own size (900).
        pred_direction_override: See `fit_rotation_translation`'s own
            docstring -- passed straight through.

    Returns:
        `(4, 2)` box corners in raw pixel space (top-left, top-right,
        bottom-right, bottom-left order), or None if `predicted_points` has
        a NaN (e.g. a frame `TinyLocalizationModel` didn't predict).
    """
    if np.isnan(predicted_points).any():
        return None
    raw_to_aligned = fit_rotation_translation(
        predicted_points, canonical_points, scale, pred_direction_override
    )
    aligned_to_raw = invert_affine(raw_to_aligned[np.newaxis])
    corners = np.array(
        [[0, 0], [box_size, 0], [box_size, box_size], [0, box_size]], dtype=np.float32
    )
    return apply_affine(corners[np.newaxis], aligned_to_raw)[0]
