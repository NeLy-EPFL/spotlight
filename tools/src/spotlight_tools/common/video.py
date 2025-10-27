import cv2
import numpy as np
from pathlib import Path
from vidgear.gears import WriteGear


def get_video_info(video_path: Path):
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
    video_writer = WriteGear(
        output=output_path, compression_mode=True, logging=logging, **output_params
    )
    for frame in frames:
        video_writer.write(frame)
    video_writer.close()
