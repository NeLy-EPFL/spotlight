import numpy as np
import pandas as pd
from itertools import product
from scipy.interpolate import interp1d


def make_spatial_grid(x_min, x_max, y_min, y_max, stride):
    """
    Create a grid of points with given stride (in mm), covering the range from
    (x_min, y_min) to (x_max, y_max).

    Parameters:
    -----------
    x_min, x_max : float
        Minimum and maximum x-coordinates
    y_min, y_max : float
        Minimum and maximum y-coordinates
    stride : float, optional
        Distance between adjacent grid points in mm

    Returns:
    --------
    pd.DataFrame
        DataFrame with columns 'x_pos_mm' and 'y_pos_mm' containing the grid points
    """
    # Create arrays of x and y coordinates
    x_coords = np.arange(x_min, x_max + stride, stride)
    y_coords = np.arange(y_min, y_max + stride, stride)

    # Create all combinations of x and y coordinates
    grid_points = list(product(x_coords, y_coords))

    # Convert to DataFrame
    grid_df = pd.DataFrame(grid_points, columns=["x_pos_mm", "y_pos_mm"])

    return grid_df


def resample_spatial_grid(spatial_grid, stage_position_log):
    """
    From a spatial grid and stage position log, find the stage positions that are
    closest to each point on the grid.

    Parameters:
    -----------
    spatial_grid : pd.DataFrame
        DataFrame with columns 'x_pos_mm' and 'y_pos_mm' containing the grid points
    stage_position_log : pd.DataFrame
        DataFrame with columns 'timestamp_us', 'x_pos_mm', and 'y_pos_mm'

    Returns:
    --------
    pd.DataFrame
        DataFrame with the same columns as stage_position_log, but only with rows
        that correspond to the positions closest to the grid points
    """
    # Initialize an empty list to store the selected stage positions
    selected_positions = []

    # For each grid point, find the closest stage position
    for _, grid_point in spatial_grid.iterrows():
        # Calculate squared distances to all stage positions
        distances = (stage_position_log["x_pos_mm"] - grid_point["x_pos_mm"]) ** 2 + (
            stage_position_log["y_pos_mm"] - grid_point["y_pos_mm"]
        ) ** 2

        # Find the index of the minimum distance
        min_idx = distances.idxmin()

        # Get the corresponding stage position row
        selected_position = stage_position_log.loc[min_idx]

        # Append to the list
        selected_positions.append(selected_position)

    # Convert to DataFrame
    resampled_stage_position_log = pd.DataFrame(selected_positions).reset_index(
        drop=True
    )

    return resampled_stage_position_log


def resample_temporal_grid(resampled_stage_position_log, images_metadata_df):
    """
    From a resampled stage position log, find the images that were taken
    closest in time to each stage position.

    Parameters:
    -----------
    resampled_stage_position_log : pd.DataFrame
        DataFrame with columns 'timestamp_us', 'x_pos_mm', and 'y_pos_mm'
    images_metadata_df : pd.DataFrame
        DataFrame with columns 'frameId', 'acquisitionTimeUs', 'receivedTimeUs', 'path', 'channel'

    Returns:
    --------
    pd.DataFrame
        DataFrame with the same columns as images_metadata_df, but only with rows
        that correspond to the images taken closest in time to the stage positions
    """
    # Initialize an empty list to store the selected images
    selected_images = []

    # For each stage position, find the closest image in time
    for _, stage_pos in resampled_stage_position_log.iterrows():
        # Calculate time differences with all images
        time_diffs = abs(
            images_metadata_df["receivedTimeUs"] - stage_pos["timestamp_us"]
        )

        # Find the index of the minimum time difference
        min_idx = time_diffs.idxmin()

        # Get the corresponding image row
        selected_image = images_metadata_df.loc[min_idx]

        # Append to the list
        selected_images.append(selected_image)

    # Convert to DataFrame
    resampled_images_metadata_df = pd.DataFrame(selected_images).reset_index(drop=True)

    return resampled_images_metadata_df


def interpolate_stage_position(stage_position_log):
    """
    Create an interpolation function that maps time to stage position.

    Parameters:
    -----------
    stage_position_log : pd.DataFrame
        DataFrame with columns 'timestamp_us', 'x_pos_mm', and 'y_pos_mm'

    Returns:
    --------
    callable
        Function that takes a time (or array of times) in microseconds and returns
        a tuple (or array of tuples) of (x_pos_mm, y_pos_mm)
    """
    # Sort by timestamp to ensure proper interpolation
    sorted_log = stage_position_log.sort_values("timestamp_us")

    # Create interpolators for x and y positions
    x_interpolator = interp1d(
        sorted_log["timestamp_us"],
        sorted_log["x_pos_mm"],
        kind="linear",
        bounds_error=False,  # Allow extrapolation
        fill_value=(
            sorted_log["x_pos_mm"].iloc[0],
            sorted_log["x_pos_mm"].iloc[-1],
        ),  # Extrapolate using edge values
    )

    y_interpolator = interp1d(
        sorted_log["timestamp_us"],
        sorted_log["y_pos_mm"],
        kind="linear",
        bounds_error=False,  # Allow extrapolation
        fill_value=(
            sorted_log["y_pos_mm"].iloc[0],
            sorted_log["y_pos_mm"].iloc[-1],
        ),  # Extrapolate using edge values
    )

    def interpolator(time_us):
        """
        Interpolate stage position at given time(s).

        Parameters:
        -----------
        time_us : float or array-like
            Time(s) in microseconds

        Returns:
        --------
        tuple or ndarray
            If time_us is a scalar, returns a tuple (x_pos_mm, y_pos_mm)
            If time_us is array-like, returns an array of shape (n, 2) where n is the length of time_us
        """
        # Interpolate x and y positions
        x_pos = x_interpolator(time_us)
        y_pos = y_interpolator(time_us)

        # Check if the input is a scalar
        if np.isscalar(time_us):
            return (float(x_pos), float(y_pos))
        else:
            return np.column_stack((x_pos, y_pos))

    return interpolator


def add_interpolated_positions(resampled_images_metadata_df, interpolator):
    """
    Add interpolated stage positions to the resampled images metadata based on receivedTimeUs.

    Parameters:
    -----------
    resampled_images_metadata_df : pd.DataFrame
        DataFrame with columns including 'receivedTimeUs'
    interpolator : callable
        Function that takes a time in microseconds and returns a tuple (x_pos_mm, y_pos_mm)

    Returns:
    --------
    pd.DataFrame
        The input DataFrame with two additional columns: 'stage_pos_x_mm' and 'stage_pos_y_mm'
    """
    # Create a copy of the input DataFrame
    result_df = resampled_images_metadata_df.copy()

    # Interpolate stage positions for each image
    positions = interpolator(result_df["receivedTimeUs"].values)

    # Add interpolated positions to the DataFrame
    if len(result_df) > 1:
        result_df["stage_pos_x_mm"] = positions[:, 0]
        result_df["stage_pos_y_mm"] = positions[:, 1]
    else:
        # Handle the case of a single row
        result_df["stage_pos_x_mm"] = positions[0]
        result_df["stage_pos_y_mm"] = positions[1]

    return result_df


def find_keyframes(stage_position_log, images_metadata_df, grid_stride=2):
    """
    Complete workflow to sample calibration images on a regular grid and
    add accurate stage positions.

    Parameters:
    -----------
    stage_position_log : pd.DataFrame
        DataFrame with columns 'timestamp_us', 'x_pos_mm', and 'y_pos_mm'
    images_metadata_df : pd.DataFrame
        DataFrame with columns 'frameId', 'acquisitionTimeUs',
        'receivedTimeUs', 'path', 'channel'
    grid_stride : float, optional
        Distance between adjacent grid points in mm (default: 2)

    Returns:
    --------
    pd.DataFrame
        DataFrame with columns from images_metadata_df plus
        'stage_pos_x_mm' and 'stage_pos_y_mm'
    """
    # Step 1: Generate spatial grid
    x_min = stage_position_log["x_pos_mm"].min()
    x_max = stage_position_log["x_pos_mm"].max()
    y_min = stage_position_log["y_pos_mm"].min()
    y_max = stage_position_log["y_pos_mm"].max()
    print(
        f"Creating {grid_stride}mm grid from ({x_min},{y_min}) to ({x_max},{y_max})..."
    )
    spatial_grid = make_spatial_grid(x_min, x_max, y_min, y_max, grid_stride)
    print(f"Created grid with {len(spatial_grid)} points")

    # Step 2: Resample stage positions to match spatial grid
    print("Resampling stage positions to match grid...")
    resampled_stage_position_log = resample_spatial_grid(
        spatial_grid, stage_position_log
    )

    # Step 3: Resample images to match resampled stage positions in time
    print("Finding closest images in time...")
    resampled_images_metadata_df = resample_temporal_grid(
        resampled_stage_position_log, images_metadata_df
    )

    # Step 4: Create stage position interpolator
    print("Creating position interpolator...")
    stage_position_interpolator = interpolate_stage_position(stage_position_log)

    # Step 5: Add interpolated stage positions to resampled images metadata
    print("Adding precise interpolated positions...")
    keyframe_metadata_df = add_interpolated_positions(
        resampled_images_metadata_df, stage_position_interpolator
    )

    print(f"Selected {len(keyframe_metadata_df)} images.")
    return keyframe_metadata_df