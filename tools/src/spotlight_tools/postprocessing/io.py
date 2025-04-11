import logging
from pathlib import Path


def check_is_directory_valid(directory: Path):
    if not directory.is_dir():
        logging.critical(f"Directory {directory} is not valid.")
        raise ValueError(f"Invalid path.")


def check_is_output_file_valid(
    output_path: Path, overwrite: bool = False, suffix: str | None = None
) -> None:
    if output_path.exists():
        if overwrite:
            output_path.unlink()
        else:
            logging.critical(
                f"Output file {output_path} already exists. Remove it or "
                f"set overwrite to True."
            )
            raise ValueError("Invalid path.")
    if not output_path.suffix in (suffix.lower(), suffix.upper()):
        logging.critical(
            f"Output file {output_path} must have {suffix} extension. "
            f"Found: {output_path}"
        )
        raise ValueError("Invalid path.")
    output_path.parent.mkdir(exist_ok=True, parents=True)


def find_files_per_frame_by_suffix(frames_dir: Path, suffix: str) -> dict[int, Path]:
    """
    Find files in the given directory with the specified suffix and
    return a dictionary mapping frame numbers to file paths. The dictionary
    keys are sorted. This function also checks that the frame numbers are
    consecutive (ie. no frame is missing).
    """
    files = list(frames_dir.glob(f"*{suffix}"))
    files_by_frame = {}
    for file in files:
        try:
            frame_number = int(file.stem.split("_")[-1])
            files_by_frame[frame_number] = file
        except ValueError:
            logging.warning(f"Could not recognize file name '{file}'. Skipping file.")
    sorted_files_by_frame = dict(sorted(files_by_frame.items()))

    last_frame_id = list(sorted_files_by_frame.keys())[-1]
    for frame_id in range(0, last_frame_id + 3, 3):
        if frame_id not in sorted_files_by_frame.keys():
            logging.error(
                f"Frame {frame_id} is missing in the directory {frames_dir}. "
                f"Skipping it."
            )
            continue
    print(
        f"Checked: {suffix} files are continuous from frame 0 to "
        f"frame {last_frame_id} in interval of 3."
    )

    return sorted_files_by_frame
