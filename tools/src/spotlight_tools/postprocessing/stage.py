import logging
import pandas as pd
from scipy.interpolate import CubicSpline
from pathlib import Path

from spotlight_tools.postprocessing.io import find_files_per_frame_by_suffix


def interp_stage_pos_at_behavior_frames(
    frames_dir: Path, stage_positions_path: Path, output_path: Path
) -> pd.DataFrame:
    """Interpolate stage positions at behavior frame timestamps.
    
    This function combines behavior frame timing metadata with stage position
    data to produce interpolated XY coordinates for each behavior frame.
    
    Args:
        frames_dir (Path): Directory containing behavior frame CSV metadata files.
        stage_positions_path (Path): Path to the stage positions CSV file.
        output_path (Path): Path where the merged metadata will be saved.
        
    Returns:
        pd.DataFrame: DataFrame with behavior frame metadata and interpolated
            stage positions (columns: behavior_frame_id, received_time_us,
            x_pos_mm_interp, y_pos_mm_interp).
    """
    logger = logging.getLogger(__name__)

    # Merge timestamps for each behavior frame
    logger.info("Merging timestamps for all behavior frames")
    files_by_frame = find_files_per_frame_by_suffix(frames_dir, ".csv", stride=3)
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
