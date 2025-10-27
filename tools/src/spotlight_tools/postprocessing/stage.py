import logging
import pandas as pd
from scipy.interpolate import CubicSpline
from pathlib import Path


def interp_stage_pos_at_behavior_frames(
    frames_dir: Path, stage_positions_path: Path, output_path: Path
) -> pd.DataFrame:
    logger = logging.getLogger(__name__)

    # Merge timestamps for each behavior frame
    logger.info("Merging timestamps for all behavior frames")
    files_by_frame = _find_files_per_frame_by_suffix(frames_dir, ".csv", stride=3)
    dataframes = [pd.read_csv(file) for file in files_by_frame.values()]
    concatenated_df = pd.concat(dataframes, ignore_index=True)
    concatenated_df.sort_values(by=["received_time_us"], inplace=True)
    concatenated_df.reset_index(drop=True, inplace=True)

    # Rename "frame_id" column to "behavior_frame_id"
    concatenated_df.rename(columns={"frame_id": "behavior_frame_id"}, inplace=True)

    # Interpolate stage positions
    logger.info("Interpolating stage positions for each behavior frame")
    spline_xpos, spline_ypos = _fit_stage_position_cubic_spline(stage_positions_path)
    timestamps = concatenated_df["received_time_us"].values
    concatenated_df["x_pos_mm_interp"] = spline_xpos(timestamps)
    concatenated_df["y_pos_mm_interp"] = spline_ypos(timestamps)

    concatenated_df.to_csv(output_path, index=False)
    return concatenated_df


def _fit_stage_position_cubic_spline(
    stage_positions_path: Path,
) -> tuple[CubicSpline, CubicSpline]:
    stage_positions_df = pd.read_csv(stage_positions_path)
    spline_xpos = CubicSpline(
        stage_positions_df["timestamp_us"], stage_positions_df["x_pos_mm"]
    )
    spline_ypos = CubicSpline(
        stage_positions_df["timestamp_us"], stage_positions_df["y_pos_mm"]
    )
    return spline_xpos, spline_ypos


def _find_files_per_frame_by_suffix(
    frames_dir: Path, suffix: str, stride: int
) -> dict[int, Path]:
    """
    Find files in the given directory with the specified suffix and
    return a dictionary mapping frame numbers to file paths. The dictionary
    keys are sorted. This function also checks that the frame numbers are
    consecutive at the specified stride (ie. no frame is missing).
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
    for frame_id in range(0, last_frame_id + stride, stride):
        if frame_id not in sorted_files_by_frame.keys():
            logging.error(
                f"Frame {frame_id} is missing in the directory {frames_dir}. "
                f"Skipping it."
            )
            continue
    print(
        f"Checked: {suffix} files are continuous from frame 0 to "
        f"frame {last_frame_id} in interval of {stride}."
    )

    return sorted_files_by_frame
