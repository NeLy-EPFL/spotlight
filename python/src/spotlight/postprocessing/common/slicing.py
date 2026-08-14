"""Shared `--frame-range` handling, for quick iteration on a slice of a
trial instead of the whole thing.
"""


def resolve_frame_range_to_file_slice(
    frame_range: tuple[int, int] | None, n_files: int
) -> tuple[int, int]:
    """`(file_start, file_end)` raw-file bounds covering `frame_range`.

    Every raw behavior file packs 3 real frames (see `behavior.py`'s module
    docstring), so `frame_range`'s own real-frame bounds are rounded
    outward to whole files: the actually-processed real-frame range can
    therefore run up to 2 frames wider than requested, on either side.

    Args:
        frame_range: `(start, end)` real frame indices, `start` inclusive,
            `end` exclusive, or `None` for the whole trial.
        n_files: Total raw file count available.

    Returns:
        `(file_start, file_end)`, or `(0, n_files)` if `frame_range` is
        `None`.
    """
    if frame_range is None:
        return 0, n_files
    start, end = frame_range
    return max(0, start // 3), min(-(-end // 3), n_files)
