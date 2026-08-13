#!/usr/bin/env python
"""Decodes a video once, sequentially, and caches every frame as JPEG at one
or more resolutions, for training pipelines (like
`spotlight.postprocessing.pose2d`) that need fast repeated access to
individual frames without paying for video seeking every time. Sequential
decode measured at ~2.6 ms/frame over a CIFS-mounted NAS video, vs.
~210 ms/frame for random-seeking into the same file (~80x), so this always
reads the whole video once, in frame order, rather than seeking to specific
indices.

Output layout, one subdirectory per requested size:
    output/450x450/frame_000000000.jpg
    output/450x450/frame_000000001.jpg
    ...
    output/900x900/frame_000000000.jpg
    ...

A size exactly matching the video's native resolution is saved without
resampling (just re-encoded as JPEG); other sizes are downsampled with
area-averaging (`cv2.INTER_AREA`), the right choice for shrinking an image
(avoids the aliasing/moire that linear/cubic interpolation can introduce).

Usage:
    python tools/spotlight_pose2d/cache_video_frames.py \\
        --input path/to/aligned_behavior_video.mkv \\
        --output bulk_data/.../frame_cache/<trial> \\
        --sizes 450x450 900x900 --quality 90
"""

import re
from pathlib import Path
from typing import Annotated

import cv2
import numpy as np
import sleap_io as sio
import tyro
from loguru import logger

SIZE_PATTERN = re.compile(r"^(\d+)x(\d+)$")


def parse_size(size: str) -> tuple[int, int]:
    """Parses a "WIDTHxHEIGHT" string, e.g. `"450x450"`, into `(width, height)`."""
    match = SIZE_PATTERN.match(size)
    if match is None:
        raise SystemExit(f"Invalid size {size!r}, expected e.g. '450x450'")
    return int(match[1]), int(match[2])


def to_2d(frame: np.ndarray) -> np.ndarray:
    """Squeezes a `(H, W, 1)` grayscale frame to `(H, W)`; passes color through."""
    if frame.ndim == 3 and frame.shape[-1] == 1:
        return frame[:, :, 0]
    return frame


def main(
    input_path: Annotated[Path, tyro.conf.arg(name="input")],
    output_dir: Annotated[Path, tyro.conf.arg(name="output")],
    sizes: list[str],
    quality: int = 90,
    override: bool = False,
) -> None:
    """Cache every frame of a video as JPEG at one or more resolutions.

    Args:
        input_path: Video to decode.
        output_dir: Directory to save one subdirectory of JPEGs per size in
            `sizes`.
        sizes: Target resolutions, each `"WIDTHxHEIGHT"` (e.g. `"450x450"`).
            A size matching the video's native resolution is saved without
            resampling.
        quality: JPEG quality, 0-100.
        override: If True, overwrite an existing size subdirectory instead
            of aborting.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    parsed_sizes = [parse_size(s) for s in sizes]
    for width, height in parsed_sizes:
        size_dir = output_dir / f"{width}x{height}"
        if size_dir.is_dir() and any(size_dir.iterdir()) and not override:
            raise SystemExit(
                f"{size_dir} already exists; pass --override to overwrite it."
            )
        size_dir.mkdir(parents=True, exist_ok=True)

    video = sio.Video(filename=str(input_path))
    n_frames, native_height, native_width, _channels = video.shape
    jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    log_every = max(1, n_frames // 20)

    for frame_idx in range(n_frames):
        frame = to_2d(video[frame_idx])
        for width, height in parsed_sizes:
            if (width, height) == (native_width, native_height):
                resized = frame
            else:
                resized = cv2.resize(
                    frame, (width, height), interpolation=cv2.INTER_AREA
                )
            frame_path = output_dir / f"{width}x{height}" / f"frame_{frame_idx:09d}.jpg"
            cv2.imwrite(str(frame_path), resized, jpeg_params)

        if frame_idx % log_every == 0:
            logger.info(f"{frame_idx}/{n_frames} frames")

    logger.info(
        f"Cached {n_frames} frames x {len(parsed_sizes)} size(s) -> {output_dir}"
    )


if __name__ == "__main__":
    tyro.cli(main)
