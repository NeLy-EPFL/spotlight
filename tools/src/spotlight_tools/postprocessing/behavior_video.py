import logging
import cv2
from vidgear.gears import WriteGear
from pathlib import Path
from tqdm import tqdm
from math import ceil

from spotlight_tools.postprocessing.io import (
    check_is_directory_valid,
    check_is_output_file_valid,
    find_files_per_frame_by_suffix,
)


def jpeg_to_mkv(
    frames_dir: Path,
    output_path: Path,
    overwrite: bool,
    play_fps: int,
    behavior_video_crf: int,
    behavior_video_preset: str,
    num_frames: int | None,
) -> None:
    print("Converting JPEG images to a single MKV video file")

    check_is_directory_valid(frames_dir)
    check_is_output_file_valid(output_path, overwrite=overwrite, suffix=".mkv")

    # Index input JPEG files
    sorted_files_by_frame = find_files_per_frame_by_suffix(frames_dir, ".jpg")
    input_files_sorted = list(sorted_files_by_frame.values())

    # Create MKV file metadata
    image = cv2.imread(str(input_files_sorted[0]))
    height, width, channels = image.shape
    if channels != 3:
        raise ValueError(
            f"Input images must have 3 channels (pseudo BGR), but found {channels} "
            f"channels."
        )

    codec = "libx264"
    output_params = {
        "-input_framerate": play_fps,
        "-c:v": codec,
        "-crf": behavior_video_crf,
        "-preset": behavior_video_preset,
        "-tune": "film",  # Optimize for high-quality video content
        "-pix_fmt": "yuv420p",
    }

    writer = WriteGear(
        output=output_path,
        compression_mode=True,
        logging=False,
        **output_params,
    )

    # Write each image to the video
    print(f"Writing {len(input_files_sorted) * 3} monochrome frames to video...")
    if num_frames:
        input_files_sorted = input_files_sorted[: int(ceil(num_frames / 3))]
    for i, path in tqdm(
        enumerate(input_files_sorted),
        total=len(input_files_sorted),
        desc="Converting frames",
    ):
        image = cv2.imread(str(path))
        if image is None:
            logging.warning(f"Could not read image '{path}'. Skipping it.")
            continue
        if image.shape != (height, width, 3):
            logging.warning(
                f"Image '{path}' has shape {image.shape}, expected shape "
                f"{(height, width, 3)}. Skipping it."
            )
            continue
        blue, green, red = cv2.split(image)
        writer.write(blue)
        writer.write(green)
        writer.write(red)

    writer.close()
    print(f"Video saved successfully to: {output_path}")
