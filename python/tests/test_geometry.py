"""Unit tests for `postprocessing.pose2d.geometry`'s affine helpers."""

import numpy as np

from spotlight.postprocessing.pose2d.geometry import apply_affine, invert_affine


def test_apply_affine_translation():
    # One frame, one node, pure translation (+10, -5).
    points = np.array([[[0.0, 0.0]]])
    matrices = np.array([[[1.0, 0.0, 10.0], [0.0, 1.0, -5.0]]])
    result = apply_affine(points, matrices)
    np.testing.assert_allclose(result, [[[10.0, -5.0]]])


def test_apply_affine_scale_and_rotate_90deg():
    # 2x scale + 90 degree rotation, one frame, two nodes.
    points = np.array([[[1.0, 0.0], [0.0, 1.0]]])
    matrices = np.array([[[0.0, -2.0, 0.0], [2.0, 0.0, 0.0]]])
    result = apply_affine(points, matrices)
    np.testing.assert_allclose(result, [[[0.0, 2.0], [-2.0, 0.0]]])


def test_apply_affine_propagates_nan():
    points = np.array([[[np.nan, 0.0]]])
    matrices = np.array([[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    result = apply_affine(points, matrices)
    assert np.isnan(result).all()


def test_invert_affine_undoes_apply_affine():
    rng = np.random.default_rng(0)
    n_frames = 5
    matrices = rng.normal(size=(n_frames, 2, 3))
    points = rng.normal(size=(n_frames, 4, 2))

    transformed = apply_affine(points, matrices)
    recovered = apply_affine(transformed, invert_affine(matrices))

    np.testing.assert_allclose(recovered, points, atol=1e-8)


def test_invert_affine_identity():
    identity = np.array([[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    result = invert_affine(identity)
    np.testing.assert_allclose(result, identity)
