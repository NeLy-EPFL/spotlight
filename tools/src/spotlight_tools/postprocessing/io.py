import logging
from pathlib import Path


def find_files_per_frame_by_suffix(
    frames_dir: Path, suffix: str, stride: int = 1
) -> dict[int, Path]:
    """
    Find files in the given directory with the specified suffix and
    return a dictionary mapping frame numbers to file paths. The dictionary
    keys are sorted. This function also checks that the frame numbers are
    consecutive at the specified stride (ie. no frame is missing).
    """
    logger = logging.getLogger(__name__)

    files = list(frames_dir.glob(f"*{suffix}"))
    files_by_frame = {}
    for file in files:
        try:
            frame_number = int(file.stem.split("_")[-1])
            files_by_frame[frame_number] = file
        except ValueError:
            logger.warning(f"Could not recognize file name '{file}'. Skipping file.")
    sorted_files_by_frame = dict(sorted(files_by_frame.items()))

    last_frame_id = list(sorted_files_by_frame.keys())[-1]
    for frame_id in range(0, last_frame_id + stride, stride):
        if frame_id not in sorted_files_by_frame.keys():
            logger.error(
                f"Frame {frame_id} is missing in the directory {frames_dir}. "
                f"Skipping it."
            )
            continue
    logger.info(
        f"Checked: {suffix} files are continuous from frame 0 to "
        f"frame {last_frame_id} in interval of {stride}."
    )

    return sorted_files_by_frame
