"""Derives a proxy "is the fly upside down or sideways" signal from the
RepVGG-A0 model's own per-keypoint confidence (`keypoint_scores` in a
`final_predictions.h5`), rather than any hand label -- the training label
`TinyOrientDataset` uses for its `flipped` target (True == flipped).

Proximal leg joints (ThC, CTr) sit close to the body and get self-occluded
specifically when the fly is flipped, unlike the more distal leg segments
(FTi/TiTa/Cl) and other nodes, which stay visible from most orientations.
Mean confidence over just those 12 nodes should separate upright from
flipped frames more sharply than an all-node average -- validated in
`scripts/spotlight_orient/flip_proxy_analysis.py`, whose pooled histogram
across the full dataset shows a clear low-confidence mode (flipped) below
`FLIPPED_THRESHOLD`, well separated from the dominant upright mode above it.
"""

import warnings

import numpy as np
from loguru import logger

PROXIMAL_LEG_NODES = [
    f"{leg}_{joint}"
    for leg in ("LF", "LM", "LH", "RF", "RM", "RH")
    for joint in ("ThC", "CTr")
]

FLIPPED_THRESHOLD = 0.5


def proximal_leg_confidence(
    keypoint_scores: np.ndarray, node_names: list[str]
) -> np.ndarray:
    """Mean confidence over `PROXIMAL_LEG_NODES`, per frame.

    Args:
        keypoint_scores: `(n_frames, n_nodes)`, e.g. from
            `spotlight_tools.spotlight_pose2d.io_utils.load_pose_h5`.
        node_names: Node name per column of `keypoint_scores`.

    Returns:
        `(n_frames,)`. NaN for a frame where every one of these 12 nodes is
        itself NaN (a hand-labeled/hand-corrected frame has no confidence at
        all, since a human click has no score -- see `load_pose_h5`).
    """
    index = {name: i for i, name in enumerate(node_names)}
    columns = [index[name] for name in PROXIMAL_LEG_NODES]
    # A hand-labeled/hand-corrected frame's row is entirely NaN (by design,
    # see the docstring below), which is exactly what np.nanmean's "Mean of
    # empty slice" RuntimeWarning is about -- expected here, so silenced.
    # np.errstate doesn't cover it: that warning comes from Python's own
    # warnings module, not numpy's floating-point error state.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Mean of empty slice")
        return np.nanmean(keypoint_scores[:, columns], axis=1)


def weighted_confidence(
    keypoint_scores: np.ndarray,
    node_names: list[str],
    primary_nodes: list[str] = PROXIMAL_LEG_NODES,
    primary_weight: float = 5.0,
    other_weight: float = 1.0,
) -> np.ndarray:
    """Weighted mean confidence over EVERY node, `primary_nodes` (default:
    the proximal leg joints ThC/CTr) weighted `primary_weight`x higher
    than everything else, per frame.

    Intuition: genuine flipping self-occludes the proximal leg joints
    specifically (see `proximal_leg_confidence`'s own docstring), but a
    fly merely close to the arena wall/edge (not actually flipped, just
    in an unusual pose there) should still show good confidence at every
    OTHER node -- the more distal leg segments, antennae, wings, thorax,
    etc. -- which a proximal-only average can't see. Blending in those
    other nodes (at a lower weight, so genuine proximal self-occlusion
    still dominates) should pull a merely-near-the-wall frame's score
    back up while leaving a genuinely flipped frame's score low, since
    that one's OTHER nodes should be struggling too.

    Args:
        keypoint_scores: `(n_frames, n_nodes)`.
        node_names: Node name per column of `keypoint_scores`.
        primary_nodes: Node names to weight `primary_weight`x.
        primary_weight: Weight for `primary_nodes`.
        other_weight: Weight for every other node.

    Returns:
        `(n_frames,)`. NaN for a frame where every node is itself NaN
        (see `proximal_leg_confidence`'s own docstring).
    """
    weights = np.array(
        [
            primary_weight if name in primary_nodes else other_weight
            for name in node_names
        ]
    )
    is_nan = np.isnan(keypoint_scores)
    numerator = np.nansum(keypoint_scores * weights, axis=1)
    denominator = np.where(is_nan, 0.0, weights).sum(axis=1)
    with np.errstate(invalid="ignore"):
        result = numerator / denominator
    result[denominator == 0] = np.nan
    return result


def valid_frames(
    keypoint_scores: np.ndarray,
    node_names: list[str],
    is_label: np.ndarray,
    confidence: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Frame indices with a usable flip-confidence proxy, and their confidence.

    Excludes hand-labeled/hand-corrected frames (`is_label`), which have no
    confidence at all (see `proximal_leg_confidence`) and so can't be given
    a flip label this way.

    Args:
        keypoint_scores: `(n_frames, n_nodes)`.
        node_names: Node name per column of `keypoint_scores`.
        is_label: `(n_frames,)`, True for hand-labeled/hand-corrected frames.
        confidence: Precomputed per-frame confidence (e.g. from
            `weighted_confidence`), or None to compute
            `proximal_leg_confidence` -- the default, and what real
            training labels use.

    Returns:
        frame_idxs: Indices into `keypoint_scores`'s frame axis.
        confidence: One value per `frame_idxs` entry.
    """
    if confidence is None:
        confidence = proximal_leg_confidence(keypoint_scores, node_names)
    n_other_nan = (~is_label & np.isnan(confidence)).sum()
    if n_other_nan:
        logger.warning(f"{n_other_nan} unexpected NaN frame(s) outside hand labels")
    valid = ~is_label & ~np.isnan(confidence)
    frame_idxs = np.flatnonzero(valid)
    return frame_idxs, confidence[frame_idxs]


def is_flipped(
    confidence: np.ndarray, threshold: float = FLIPPED_THRESHOLD
) -> np.ndarray:
    """True where `confidence` (from `proximal_leg_confidence`/`valid_frames`)
    indicates the fly is flipped (upside down or sideways): low confidence
    at the proximal leg joints means they're self-occluded, i.e. the fly
    probably isn't right-side up.
    """
    return confidence < threshold
