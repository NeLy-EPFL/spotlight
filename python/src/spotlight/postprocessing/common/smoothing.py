"""Shared time-domain smoothing helpers, for CLI parameters expressed in
seconds (portable across recordings at different `behavior_fps`) rather than
raw frame counts.
"""

import numpy as np
from scipy.ndimage import binary_closing, binary_opening, gaussian_filter1d


def seconds_to_odd_frames(seconds: float, fps: float) -> int:
    """Convert a duration in seconds to an odd-sized frame window.

    A symmetric structuring element (morphological open/close) needs an odd
    size to have a well-defined center frame. `-1` means "disabled" and
    passes through unchanged (see `smooth_acceptance_mask`).

    Args:
        seconds: Duration in seconds, or `-1` to disable.
        fps: Behavior recording frame rate.

    Returns:
        Odd frame count (minimum 1), or `-1`.
    """
    if seconds == -1:
        return -1
    frames = max(1, round(seconds * fps))
    return frames if frames % 2 == 1 else frames + 1


def seconds_to_frames(seconds: float, fps: float) -> float:
    """Convert a Gaussian smoothing sigma from seconds to frames. Unlike
    `seconds_to_odd_frames`, no rounding: `gaussian_filter1d` takes a plain
    float sigma. `-1` means "disabled" and passes through unchanged.
    """
    return seconds if seconds == -1 else seconds * fps


def smooth_unit_vectors(vectors: np.ndarray, sigma: float) -> np.ndarray:
    """Gaussian-smooths a `(n_frames, d)` batch of unit vectors over their
    leading (time) axis, one component at a time, then renormalizes:
    sidesteps the wraparound a naive angle average would hit near +/-180
    degrees. Used for camera-heading smoothing (a synthetic-3D-panel
    camera's yaw-tracking direction), not any real alignment/IK data.

    Args:
        vectors: `(n_frames, d)`, NaN-free (call once per contiguous run,
            not across a gap: smoothing through one would blend
            unrelated stretches at the seam).
        sigma: Gaussian sigma, in frames. `-1` returns `vectors` unchanged.
    """
    if sigma == -1:
        return vectors
    smoothed = np.stack(
        [
            gaussian_filter1d(vectors[:, a], sigma=sigma)
            for a in range(vectors.shape[1])
        ],
        axis=1,
    )
    norms = np.linalg.norm(smoothed, axis=1, keepdims=True)
    return smoothed / np.clip(norms, 1e-8, None)


def smooth_acceptance_mask(mask: np.ndarray, window: int) -> np.ndarray:
    """Denoises a boolean per-frame acceptance time series with a binary
    opening (drops isolated accepted spikes) followed by a closing (fills
    isolated rejected gaps inside a longer accepted stretch), both using a
    `window`-sized 1D structuring element. `window == -1` skips this
    denoising step entirely (the raw per-frame mask is used as-is).
    """
    if window == -1:
        return mask
    structure = np.ones(window, dtype=bool)
    mask = binary_opening(mask, structure=structure)
    mask = binary_closing(mask, structure=structure)
    return mask
