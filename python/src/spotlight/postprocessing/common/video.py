"""Video I/O helpers: read metadata, write encoded video via `pvio`.

`pvio.write_frames_to_video` wants a materialized, re-iterable frame list
(it calls `len()` and may iterate twice, for the NVENC->libx264 fallback),
not a true incremental stream. Holding a whole trial's frames in memory to
satisfy that isn't viable (tens of GB at full trial length), so
`StreamingVideoWriter` below buffers each chunk to a scratch directory as
JPEGs as the pipeline produces them (bounded memory, one chunk at a time)
and only encodes at the very end via `pvio.write_image_paths_to_video`,
which reads images lazily from disk. This still satisfies the actual goal
(model inference always runs on in-memory arrays, strictly before any
frame is bound into a video): it just uses a JPEG intermediate for the
encode step itself, exactly as the old vidgear-based pipeline already did
internally (`decode_and_align_all_behavior_frames`'s own temp-dir pattern).
"""

import logging
import shutil
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pvio

logger = logging.getLogger(__name__)

# pvio's `quality` knob is the same 0-51 H.264 quantiser scale for both the
# libx264 CRF path (CPU) and the NVENC constant-QP path (GPU): see
# `pvio.io.write_frames_to_video`'s own docstring. A target CRF therefore
# doubles directly as the GPU-path quality value with no separate mapping.
DEFAULT_MODE = "auto"


def pad_to_macroblock(frame: np.ndarray, multiple: int = 16) -> np.ndarray:
    """Pads `frame` with black on the bottom/right to the next multiple of
    `multiple` in both dimensions.

    H.264 requires both dimensions to be a multiple of its macroblock size
    (16); imageio/ffmpeg otherwise silently stretch-resizes every frame to
    the next multiple to cope (with a `FFMPEG_WRITER WARNING` on every
    call), which both spams the log and applies an unrequested resample.
    Padding ourselves to the exact target size upfront avoids both.
    """
    h, w = frame.shape[:2]
    new_h = -(-h // multiple) * multiple
    new_w = -(-w // multiple) * multiple
    if new_h == h and new_w == w:
        return frame
    return cv2.copyMakeBorder(
        frame, 0, new_h - h, 0, new_w - w, cv2.BORDER_CONSTANT, value=0
    )


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
    fps: float,
    crf: int,
    preset: str | None = None,
    mode: str = DEFAULT_MODE,
    logging: bool = True,
) -> None:
    """Write an in-memory frame list to a video file via `pvio` (one shot,
    no chunking; use `StreamingVideoWriter` for a pipeline that produces
    frames incrementally)."""
    frames = [pad_to_macroblock(f) for f in frames]
    pvio.write_frames_to_video(
        str(output_path), frames, fps, mode=mode, quality=crf, preset=preset,
        quiet=not logging,
    )  # fmt: skip


class StreamingVideoWriter:
    """Bounded-memory video writer: buffer chunks to scratch JPEGs as they
    arrive, encode once at `.close()`.

    Usage:
        writer = StreamingVideoWriter(output_path, fps=12, crf=18)
        for chunk in chunks:
            writer.write_chunk(chunk)  # chunk: list[np.ndarray] or (N,H,W[,C]) array
        writer.close()
    """

    def __init__(
        self,
        output_path: Path,
        fps: float,
        crf: int,
        preset: str | None = None,
        mode: str = DEFAULT_MODE,
    ) -> None:
        self.output_path = Path(output_path)
        self.fps = fps
        self.crf = crf
        self.preset = preset
        self.mode = mode
        self._tmpdir = tempfile.mkdtemp(prefix="streaming_video_")
        self._frame_idx = 0

    # Scratch-file quality only (the real output quality is `self.crf`,
    # applied at the actual encode below): 85 measured ~17% faster to
    # write than cv2's default 95 and produces a smaller intermediate
    # (faster for pvio to read back too), with no visible effect on the
    # final video since this file is immediately re-encoded, never viewed.
    _SCRATCH_JPEG_QUALITY = 85

    def write_chunk(self, frames: np.ndarray | list[np.ndarray]) -> None:
        for frame in frames:
            path = Path(self._tmpdir) / f"frame_{self._frame_idx:09d}.jpg"
            cv2.imwrite(
                str(path), pad_to_macroblock(frame),
                [cv2.IMWRITE_JPEG_QUALITY, self._SCRATCH_JPEG_QUALITY],
            )  # fmt: skip
            self._frame_idx += 1

    def close(self) -> None:
        try:
            if self._frame_idx == 0:
                raise ValueError(f"No frames were written for {self.output_path}")
            paths = sorted(Path(self._tmpdir).glob("frame_*.jpg"))
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            # `quiet=True`: pvio's own progress bar (tqdm, when stdout is a
            # TTY) has no label saying which video it's for, and pvio ties
            # its `log_interval` periodic-logging alternative to `quiet=
            # False` too, so there's no way to get labeled/periodic
            # progress out of it here; this line is the substitute.
            logger.info(
                f"Encoding {self.output_path.name} ({self._frame_idx} frames)..."
            )
            pvio.write_image_paths_to_video(
                str(self.output_path), paths, self.fps,
                mode=self.mode, quality=self.crf, preset=self.preset,
                quiet=True,
            )  # fmt: skip
        finally:
            shutil.rmtree(self._tmpdir, ignore_errors=True)
