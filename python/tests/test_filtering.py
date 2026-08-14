"""Unit tests for `common.filtering`."""

import numpy as np
import pytest

from spotlight.postprocessing.common import (
    seconds_to_frames,
    morph_denoise_1d_mask,
    smooth_unit_vectors,
)


class TestSecondsToFramesOddInt:
    def test_disabled_passes_through(self):
        assert seconds_to_frames(-1, fps=396.0, odd_int=True) == -1

    def test_rounds_up_to_odd(self):
        # 0.01s * 396fps = 3.96 -> rounds to 4 -> bumped to the next odd, 5.
        assert seconds_to_frames(0.01, fps=396.0, odd_int=True) == 5

    def test_already_odd_stays_unchanged(self):
        # 0.005s * 396fps = 1.98 -> rounds to 2 -> bumped to 3.
        assert seconds_to_frames(0.005, fps=396.0, odd_int=True) == 3

    def test_minimum_is_one(self):
        assert seconds_to_frames(0.0, fps=396.0, odd_int=True) == 1


class TestSecondsToFrames:
    def test_disabled_passes_through(self):
        assert seconds_to_frames(-1, fps=396.0) == -1

    def test_scales_by_fps_without_rounding(self):
        assert seconds_to_frames(0.015, fps=396.0) == pytest.approx(5.94)


class TestSmoothUnitVectors:
    def test_disabled_passes_through_unchanged(self):
        vectors = np.array([[1.0, 0.0], [0.0, 1.0]])
        result = smooth_unit_vectors(vectors, sigma=-1)
        np.testing.assert_array_equal(result, vectors)

    def test_output_stays_unit_norm(self):
        rng = np.random.default_rng(0)
        angles = np.cumsum(rng.normal(scale=0.1, size=50))
        vectors = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        result = smooth_unit_vectors(vectors, sigma=3.0)
        norms = np.linalg.norm(result, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6)

    def test_constant_input_is_unchanged(self):
        vectors = np.tile([0.6, 0.8], (10, 1))  # already a unit vector
        result = smooth_unit_vectors(vectors, sigma=2.0)
        np.testing.assert_allclose(result, vectors, atol=1e-6)

    def test_avoids_wraparound_cancellation(self):
        # Two headings just on either side of +/-180 degrees (~179.4 and
        # ~-179.4 degrees), i.e. two vectors both pointing almost due
        # -x. Averaging the raw angle values would wrap around and land
        # near 0 degrees (+x): exactly backwards. Smoothing x/y
        # separately then renormalizing instead keeps the result pointing
        # the same way as both inputs.
        vectors = np.array([[-1.0, 0.01], [-1.0, -0.01]])
        result = smooth_unit_vectors(vectors, sigma=0.5)
        norms = np.linalg.norm(result, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6)
        assert (result[:, 0] < -0.9).all()  # still pointing in -x, not +x


class TestMorphDenoise1dMask:
    def test_disabled_passes_through(self):
        mask = np.array([True, False, True])
        result = morph_denoise_1d_mask(mask, window=-1)
        np.testing.assert_array_equal(result, mask)

    def test_opening_drops_isolated_interior_spike(self):
        mask = np.zeros(20, dtype=bool)
        mask[10] = True
        result = morph_denoise_1d_mask(mask, window=5)
        assert not result.any()

    def test_closing_fills_isolated_interior_gap(self):
        mask = np.ones(20, dtype=bool)
        mask[10] = False
        result = morph_denoise_1d_mask(mask, window=5)
        assert result[2:18].all()  # edges excluded, see test below

    def test_edges_are_always_rejected(self):
        # `binary_opening`/`binary_closing` treat out-of-bounds neighbors
        # as False (see scripts/postprocessing/README.md's history section
        # for a production bug this once caused elsewhere), so the
        # first/last `window // 2` frames are eroded away regardless of
        # content. Harmless for a real many-thousand-frame trial; worth
        # pinning down explicitly here since it's easy to get backwards.
        mask = np.ones(20, dtype=bool)
        result = morph_denoise_1d_mask(mask, window=5)
        assert not result[:2].any()
        assert not result[-2:].any()
