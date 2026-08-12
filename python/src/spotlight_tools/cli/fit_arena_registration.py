"""
Fit the linear (stage, pixel) <-> physical model from a registration scan
captured by `run-arena-registration-scan`.

Inputs (under `<arena_dir>`):
    - metadata.yaml                              (arena-space AprilTag corners)
    - mapping_scan/apriltag<id>_img<i>.jpg       (10 frames per tag)
    - mapping_scan/apriltag_stage_positions.csv  (stage pose per frame)

Outputs (under `<arena_dir>/model/`):
    - calibration_points.csv     per-corner detections used for the fit
    - calibration_result.yaml    fitted forward and inverse coefficients
    - diagnostics.png            fit diagnostics
    - detections/...             per-image detection overlays (optional)
"""

import os
from typing import Annotated

import tyro

from spotlight_tools.arena import APRILTAG_FAMILY, fit_arena_registration


def fit_arena_registration_cli(
    arena_dir: Annotated[str, tyro.conf.arg(aliases=["-a"])],
    apriltag_family: str = APRILTAG_FAMILY,
    min_decision_margin: float = 20.0,
    quad_decimate: float = 4.0,
    mad_threshold: float = 3.0,
    ransac_residual_threshold: float = 0.5,
    ransac_max_trials: int = 1000,
    visualize_detections: bool = False,
) -> None:
    """Fit the arena registration model.

    Args:
        arena_dir: Path to the arena directory (containing metadata.yaml
            and mapping_scan/). The script writes its output under
            `<arena_dir>/model/`.
        apriltag_family: AprilTag dictionary used when the registration
            board was generated.
        min_decision_margin: Drop AprilTag detections with
            decision_margin below this value. tag16h5 has a high false-
            positive rate; valid registration-scan detections sit at
            tens to hundreds while noise is typically below ~5.
        quad_decimate: Decimation factor for the AprilTag quad detector.
            Lower values (e.g. 1.0) improve detection at small tag sizes
            at the cost of speed; higher values (e.g. 4.0) are faster.
        mad_threshold: Per-burst outlier cutoff for corner detections.
            For each (apriltag_id, corner_id), the 10 frames yield 10
            pixel positions that should be nearly identical (the stage
            is stationary). MAD = median absolute deviation, a
            robust-to-outliers measure of scatter. A detection is
            dropped when its distance from the per-group median exceeds
            `mad_threshold * 1.4826 * MAD` (the 1.4826 factor makes the
            MAD scale comparable to a Gaussian standard deviation, so
            `mad_threshold` reads like a sigma-equivalent cutoff: e.g.
            3.0 ~= "three sigmas").
        ransac_residual_threshold: RANSAC inlier threshold (mm).
        ransac_max_trials: Number of RANSAC trials per axis.
        visualize_detections: If True, also save per-image AprilTag
            detection overlays under
            `<arena_dir>/model/detections/`.
    """
    fit_arena_registration(
        arena_dir=arena_dir,
        family=apriltag_family,
        min_decision_margin=min_decision_margin,
        quad_decimate=quad_decimate,
        mad_threshold=mad_threshold,
        ransac_residual_threshold=ransac_residual_threshold,
        ransac_max_trials=ransac_max_trials,
        visualize_detections=visualize_detections,
    )


def main() -> None:
    tyro.cli(fit_arena_registration_cli)
    # pupil_apriltags has a double-free in its C destructor; skip Python
    # teardown to avoid a crash on exit. All output has been flushed by here.
    os._exit(0)


if __name__ == "__main__":
    main()
