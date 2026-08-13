"""Round-trip test for `invkin.mapping`'s px<->mm conversion, using the
real (fitted) calibration file from the `mini_recording` fixture.
"""

import numpy as np

from spotlight.calibration.mapper import SpotlightPositionMapper
from spotlight.postprocessing.invkin.mapping import convert_mm_to_px, convert_px_to_mm


def test_px_to_mm_and_back_recovers_original_points(mini_recording_dir):
    calibration_path = (
        mini_recording_dir / "metadata" / "calibration_parameters_behavior.yaml"
    )
    mapper = SpotlightPositionMapper(calibration_path)

    rng = np.random.default_rng(0)
    n_frames, n_nodes = 5, 4
    aligned_px = rng.uniform(100, 800, size=(n_frames, n_nodes, 2))
    # Identity raw<->aligned transform: isolates the mm/px calibration
    # round-trip from the (separately tested, see test_geometry.py)
    # affine apply/invert step.
    transform_matrices = np.tile(np.eye(2, 3), (n_frames, 1, 1))
    stage_positions_mm = rng.uniform(50, 150, size=(n_frames, 2))

    physical_mm = convert_px_to_mm(
        aligned_px, transform_matrices, stage_positions_mm, mapper
    )
    recovered_px = convert_mm_to_px(
        physical_mm, transform_matrices, stage_positions_mm, mapper
    )

    np.testing.assert_allclose(recovered_px, aligned_px, atol=1e-6)


def test_px_to_mm_propagates_nan(mini_recording_dir):
    calibration_path = (
        mini_recording_dir / "metadata" / "calibration_parameters_behavior.yaml"
    )
    mapper = SpotlightPositionMapper(calibration_path)

    aligned_px = np.array([[[np.nan, np.nan]]])
    transform_matrices = np.eye(2, 3)[np.newaxis]
    stage_positions_mm = np.array([[100.0, 100.0]])

    physical_mm = convert_px_to_mm(
        aligned_px, transform_matrices, stage_positions_mm, mapper
    )
    assert np.isnan(physical_mm).all()
