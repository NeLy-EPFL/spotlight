"""Unit tests for `common.frame_range.resolve_frame_range_to_file_slice`."""

import pytest

from spotlight.postprocessing.common.frame_range import (
    resolve_frame_range_to_file_slice,
)


def test_none_covers_whole_trial():
    assert resolve_frame_range_to_file_slice(None, n_files=100) == (0, 100)


@pytest.mark.parametrize(
    "frame_range, n_files, expected",
    [
        # Exact multiples of 3: no outward rounding needed.
        ((0, 3), 100, (0, 1)),
        ((3, 9), 100, (1, 3)),
        # Non-multiples round outward to whole files.
        ((1, 4), 100, (0, 2)),
        ((2, 5), 100, (0, 2)),
        ((4, 4), 100, (1, 2)),
        # Clipped to the available file count.
        ((0, 10_000), 5, (0, 5)),
        # Start clipped at 0 even if negative-ish math would undershoot.
        ((0, 1), 5, (0, 1)),
    ],
)
def test_rounds_outward_to_whole_files(frame_range, n_files, expected):
    assert resolve_frame_range_to_file_slice(frame_range, n_files) == expected
