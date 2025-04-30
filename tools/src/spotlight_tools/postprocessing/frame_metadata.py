import pandas as pd
from scipy.interpolate import CubicSpline
from pathlib import Path

from spotlight_tools.postprocessing.io import (
    check_is_directory_valid,
    check_is_output_file_valid,
    find_files_per_frame_by_suffix,
)


def consolidate_behavior_acquisition_times_and_stage_positions(
    frames_dir: Path, stage_positions_path: Path, output_path: Path, overwrite: bool
) -> None:
    check_is_directory_valid(frames_dir)
    check_is_output_file_valid(output_path, overwrite=overwrite, suffix=".csv")

    # Merge timestamps for each behavior frame
    print("Merging timestamps for all behavior frames")
    sorted_files_by_frame = find_files_per_frame_by_suffix(frames_dir, ".csv")
    dataframes = [pd.read_csv(file) for file in sorted_files_by_frame.values()]
    concatenated_df = pd.concat(dataframes, ignore_index=True)
    concatenated_df.sort_values(by=["received_time_us"], inplace=True)
    concatenated_df.reset_index(drop=True, inplace=True)

    # Interpolate stage positions
    print("Interpolating stage positions for each behavior frame")
    spline_xpos, spline_ypos = fit_stage_position_cubic_spline(stage_positions_path)
    timestamps = concatenated_df["received_time_us"].values
    concatenated_df["x_pos_mm_interp"] = spline_xpos(timestamps)
    concatenated_df["y_pos_mm_interp"] = spline_ypos(timestamps)

    concatenated_df.to_csv(output_path, index=False)


def fit_stage_position_cubic_spline(
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
