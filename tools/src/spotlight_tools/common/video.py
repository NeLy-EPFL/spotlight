"""Video I/O helpers: read metadata, write encoded video via vidgear/ffmpeg."""

import os
import tempfile

import cv2
import numpy as np
from pathlib import Path
from vidgear.gears import WriteGear
from vidgear.gears import writegear as _writegear


def _check_write_access(path: str, is_windows: bool = False, logging: bool = False) -> bool:
    """Truthful write-access check that actually probes the directory.

    vidgear's default *nix implementation only inspects the directory's owner /
    group / world permission bits, which gives false negatives on network mounts
    (e.g. NFS with root_squash) where the on-disk bits say `root:root 0755` but
    the mount still permits writes. Here we simply attempt to create a temp file,
    which is what vidgear already does on Windows.
    """
    try:
        directory = Path(path)
        if not (directory.exists() and directory.is_dir()):
            return False
        fd, tmp_name = tempfile.mkstemp(dir=str(directory))
        os.close(fd)
        os.remove(tmp_name)
        return True
    except OSError:
        return False


# Patch vidgear's overly-strict write-access check (see _check_write_access).
_writegear.check_WriteAccess = _check_write_access


def get_video_info(video_path: Path):
    """Get frame width, height, and total frame count of a video file."""
    video = cv2.VideoCapture(str(video_path))
    if not video.isOpened():
        raise RuntimeError(f"Error: Could not open video {video_path}.")

    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    video.release()

    return width, height, frame_count


def write_video(
    output_path: Path,
    frames: list[np.ndarray],
    fps: int,
    crf: int,
    preset: str,
    logging: bool = False,
    **kwargs,
):
    """Write a sequence of frames to a video file using ffmpeg/libx264. Additional
    arguments in kwargs will be passed to the vidgear WriteGear upon init."""
    video_writer = get_video_writer(
        output_path, fps, crf, preset, logging=logging, **kwargs
    )
    for frame in frames:
        video_writer.write(frame)
    video_writer.close()


def get_video_writer(
    output_path: Path, fps: int, crf: int, preset: str, logging: bool = False, **kwargs
):
    """Create a vidgear WriteGear video writer configured for encoding with libx264."""
    codec = "libx264"
    output_params = {
        "-input_framerate": fps,
        "-c:v": codec,
        "-crf": crf,
        "-preset": preset,
        "-tune": "film",  # Optimize for high-quality video content
        "-pix_fmt": "yuv420p",
        **kwargs,
    }
    return WriteGear(
        output=str(output_path), compression_mode=True, logging=logging, **output_params
    )
