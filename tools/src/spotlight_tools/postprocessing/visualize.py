import logging
import numpy as np
import pandas as pd
import cv2
import matplotlib.pyplot as plt
import yaml
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.colors import Normalize
from matplotlib.collections import LineCollection
from matplotlib import cm
from pathlib import Path
from tqdm import tqdm, trange

from spotlight_tools.common.video import get_video_info
from spotlight_tools.common.dataloader import (
    load_processed_muscle_image,
    load_processed_behavior_frame,
)


def visualize_stage_trajectory(
    consolidated_metadata_df: pd.DataFrame,
) -> tuple[Figure, Axes]:
    """Create a matplotlib figure showing the XY trajectory of the stage.

    The trajectory is colored by time (seconds) starting at 0. The function
    returns the created (fig, ax) so callers can further customize or save
    the figure.

    Args:
        consolidated_metadata_df (pd.DataFrame): DataFrame containing at least
            the columns `x_pos_mm_interp`, `y_pos_mm_interp` and
            `received_time_us` used to plot the trajectory and color by time.

    Returns:
        tuple[Figure, Axes]: The matplotlib Figure and Axes containing the plot.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Stage position trajectory")
    ax.set_xlabel("X Position (mm)")
    ax.set_ylabel("Y Position (mm)")
    ax.set_aspect("equal", adjustable="box")

    x_pos = consolidated_metadata_df["x_pos_mm_interp"].values
    y_pos = consolidated_metadata_df["y_pos_mm_interp"].values

    points = np.array([x_pos, y_pos]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    colormap_name = "gnuplot"
    cmap = cm.get_cmap(colormap_name)

    times = (
        consolidated_metadata_df["received_time_us"].values / 1e6
    )  # Convert to seconds
    times = times - times[0]  # Normalize to start at 0
    norm = Normalize(vmin=times.min(), vmax=times.max())
    lc = LineCollection(segments, cmap=cmap, norm=norm)

    lc.set_array(times[:-1])
    lc.set_linewidth(2)
    line = ax.add_collection(lc)

    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label("Time (s)")
    ax.grid(True, linestyle="--", alpha=0.7)

    margin_x = 0.05 * (x_pos.max() - x_pos.min())
    margin_y = 0.05 * (y_pos.max() - y_pos.min())
    margin = max(margin_x, margin_y)
    ax.set_xlim(x_pos.min() - margin, x_pos.max() + margin)
    ax.set_ylim(y_pos.min() - margin, y_pos.max() + margin)

    return fig, ax


def generate_summary_video(
    recording_dir: Path,
    draw_pose: bool,
    draw_muscle: bool,
    muscle_vrange: tuple[int, int] | None = None,
    muscle_vrange_quantiles: tuple[float, float] | None = (97.0, 99.995),
    muscle_vrange_quantiles_sample_rate: float = 0.05,
    play_fps: int = 30,
    num_frames: int | None = None,
    overwrite: bool = False,
) -> None:
    """Generate a summary video that combines behavior frames, pose overlays and
    muscle images for quick visual inspection.

    The generated video is written to ``<recording_dir>/processed/summary_video.mp4``.
    The output layout depends on ``draw_muscle``:
      - If ``draw_muscle=True`` the final frame contains three panels: behavior
        grayscale, behavior with pose overlay, and muscle image (mapped to the
        green channel).
      - If ``draw_muscle=False`` the final frame contains two panels: behavior
        grayscale and behavior with pose overlay.

    Args:
        recording_dir (Path): Path to the recording directory (expects a
            ``processed`` subdirectory with preprocessed data).
        draw_pose (bool): If True, overlay 2D pose keypoints (loaded from
            ``processed/pose_2d.npz``) onto the behavior frames.
        draw_muscle (bool): If True, include muscle image panels in the output
            video (loaded from ``processed/muscle_images``).
        muscle_vrange (tuple[int, int] | None): Optional (vmin, vmax) used to
            normalize muscle images for display. If None, an adaptive range is
            computed from sampled images using ``muscle_vrange_quantiles``.
        muscle_vrange_quantiles (tuple[float, float] | None): Percentiles used
            to determine an adaptive muscle dynamic range when ``muscle_vrange``
            is None. Expressed as percentages (e.g. ``(97.0, 99.995)``).
        muscle_vrange_quantiles_sample_rate (float): Fraction of muscle images to
            sample when computing the adaptive range (e.g. 0.05 samples 5% of
            images).
        play_fps (int): Frames-per-second for the output video.
        num_frames (int | None): If set, limit processing to the first
            ``num_frames`` behavior frames (useful for testing).
        overwrite (bool): If False and the output file already exists the
            function raises a RuntimeError; if True it will overwrite.

    Raises:
        RuntimeError: If required preprocessed files are missing, the output file
            already exists and ``overwrite=False``, or the behavior video cannot
            be opened.

    Returns:
        None: The function writes the video file to disk and does not return
        a value.
    """
    experiment_metadata, recorder_config = _load_metadata(recording_dir)
    processed_dir = recording_dir / "processed"
    behavior_video_path = processed_dir / "behavior_video.mkv"
    muscle_images_dir = processed_dir / "muscle_images"
    sync_ratio = experiment_metadata["muscle_sync_ratio"]

    output_path = processed_dir / "summary_video.mp4"
    if output_path.is_file() and not overwrite:
        logging.error(
            f"Output video {output_path} already exists. Change the output path "
            f"or use `overwrite=True`."
        )
        raise RuntimeError("Output video already exists.")

    # Index files to be used
    width, height, num_behavior_images = get_video_info(behavior_video_path)
    if draw_muscle:
        muscle_images_paths = sorted(list(muscle_images_dir.glob("*.tif")))
        num_behavior_images_usable, num_muscle_images_usable = (
            _calculate_num_usable_images(
                num_behavior_images, len(muscle_images_paths), sync_ratio
            )
        )
        if num_frames is not None:
            num_behavior_images_usable = min(num_behavior_images_usable, num_frames)
            num_muscle_images_usable = num_behavior_images_usable // sync_ratio
        muscle_images_paths = muscle_images_paths[:num_muscle_images_usable]
    else:
        num_behavior_images_usable = num_behavior_images
        if num_frames is not None:
            num_behavior_images_usable = min(num_behavior_images_usable, num_frames)

    # Check if nominal width of image is incorrect
    if draw_muscle:
        _muscle_img_sample = cv2.imread(
            str(muscle_images_paths[0]), cv2.IMREAD_UNCHANGED
        )
        assert len(_muscle_img_sample.shape) == 2, "Muscle image is not single channel"
        muscle_width = _muscle_img_sample.shape[1]
        if muscle_width != width:
            _behavior_image_width = width
            width = min(width, muscle_width)
            logging.warning(
                f"Behavior images have a width of {_behavior_image_width} pixels, but "
                f"muscle images have a width of {muscle_width} pixels. This is likely "
                f"because the user set the desired ROI height is not allowed by the "
                f"camera, so the camera rounded it to the nearest allowed value. (Note: "
                f"the canonically oriented behavior images is 90-degree rotated and "
                f"flipped compared to how the camera sensor acquires them; hence it's "
                f"the ROI height parameter that matters. Taking the lower of the widths."
            )

    # If necessary, determine vmin and vmax of muscle images for visualization
    if draw_muscle and muscle_vrange is None:
        unwarped_muscle_images_paths = sorted(
            list(recording_dir.glob("muscle_images/*.tif"))
        )
        muscle_vrange = _determine_adaptive_muscle_vrange(
            muscle_vrange_quantiles,
            muscle_vrange_quantiles_sample_rate,
            unwarped_muscle_images_paths,
        )
        print(f"Determined adaptive muscle value range: {muscle_vrange}.")

    # Load 2D pose estimation data
    if draw_pose:
        pose_2d_path = processed_dir / "pose_2d.npz"
        pose_2d_data = np.load(pose_2d_path)["nodes_xy"]

    # Initialize behavior image reader
    behavior_video_reader = cv2.VideoCapture(str(behavior_video_path))
    if not behavior_video_reader.isOpened():
        logging.error(f"Error: Could not open behavior video {behavior_video_path}.")
        raise RuntimeError("Could not open behavior video.")

    # Initialize video writer
    num_panels = 1
    if draw_pose:
        num_panels += 1
    if draw_muscle:
        num_panels += 1
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        play_fps,
        (width * num_panels, height),
        True,
    )

    # Process each frame
    curr_muscle_frame_id = None
    muscle_image = None
    for i in trange(
        num_behavior_images_usable,
        desc="Generating summary video",
        disable=None,
    ):
        # Behavior image
        ret, behavior_image = behavior_video_reader.read()
        if not ret:
            logging.error(f"Error: Could not read frame {i} from behavior video.")
            break
        behavior_image_original = behavior_image[:, :width, 0]

        # Overlay 2D pose estimation
        if draw_pose:
            behavior_image_with_pose = behavior_image[:, :width, :].copy()
            nodes_xy = pose_2d_data[i, :, :]
            draw_fly_3keypoints(behavior_image_with_pose, nodes_xy)

        # Muscle image
        if draw_muscle:
            muscle_frame_id = i // sync_ratio
            if muscle_frame_id != curr_muscle_frame_id:
                muscle_image_path = muscle_images_paths[muscle_frame_id]
                muscle_image = cv2.imread(str(muscle_image_path), cv2.IMREAD_UNCHANGED)
                muscle_image = muscle_image[:, :width]
                curr_muscle_frame_id = muscle_frame_id

                muscle_image = muscle_image.astype(np.float32).clip(*muscle_vrange)
                muscle_image = (
                    255
                    * (muscle_image - muscle_vrange[0])
                    / (muscle_vrange[1] - muscle_vrange[0])
                ).astype(np.uint8)

        # Make final frame (by concatenating behavior and muscle) and write to video
        final_frame = np.zeros((height, width * num_panels, 3), dtype=np.uint8)
        curr_col_start = 0
        # Plot behavior original
        columns = slice(curr_col_start, curr_col_start + width)
        final_frame[:, columns, :] = behavior_image_original[:, :, None]
        curr_col_start += width
        # Plot behavior with pose (if requested)
        if draw_pose:
            columns = slice(curr_col_start, curr_col_start + width)
            final_frame[:, columns, :] = behavior_image_with_pose[:, :]
            curr_col_start += width
        # Plot muscle image (if requested)
        if draw_muscle:
            columns = slice(curr_col_start, curr_col_start + width)
            final_frame[:, columns, 1] = muscle_image  # green channel
            curr_col_start += width
        writer.write(final_frame)

    writer.release()


def draw_fly_3keypoints(
    image,
    nodes_xy,
    keypoint_colors=[(31, 119, 180), (255, 127, 14), (44, 160, 44)],
    keypoint_radius=10,
    line_color=(128, 128, 128),
    line_width=4,
):
    """
    Draws a fly skeleton with 3 keypoints (neck, thorax, abdomen) on the given image.

    Args:
        image (np.ndarray): The image on which to draw the skeleton.
        nodes_xy (np.ndarray): An array of shape (3, 2) containing the (x, y)
            coordinates of the 3 keypoints.
        keypoint_colors (list of tuple, optional): List of RGB colors for each keypoint.
        keypoint_radius (int, optional): Radius of the circles representing keypoints.
        line_color (tuple, optional): RGB color for the lines connecting keypoints.
        line_width (int, optional): Width of the lines connecting keypoints.

    Returns:
        None: The function modifies the input image in place.
    """
    for j in range(1, nodes_xy.shape[0]):
        prev_pt_xy = nodes_xy[j - 1, :]
        curr_pt_xy = nodes_xy[j, :]
        if np.any(np.isnan(prev_pt_xy)) or np.any(np.isnan(curr_pt_xy)):
            continue  # Skip if any coordinate is NaN
        cv2.line(
            image,
            (int(round(prev_pt_xy[0])), int(round(prev_pt_xy[1]))),
            (int(round(curr_pt_xy[0])), int(round(curr_pt_xy[1]))),
            line_color[::-1],  # Convert RGB to BGR
            line_width,
        )
    for j in range(nodes_xy.shape[0]):
        curr_pt_xy = nodes_xy[j, :]
        if np.any(np.isnan(curr_pt_xy)):
            continue  # Skip if any coordinate is NaN
        cv2.circle(
            image,
            (int(round(curr_pt_xy[0])), int(round(curr_pt_xy[1]))),
            keypoint_radius,
            keypoint_colors[j][::-1],  # Convert RGB to BGR
            -1,
        )


def _load_metadata(recording_dir: Path):
    """Load experiment parameters and recorder configuration YAML files.

    Args:
        recording_dir (Path): Path to the recording directory containing a
            `metadata` subdirectory with `experiment_parameters.yaml` and
            `recorder_config.yaml` files.

    Returns:
        tuple: (experiment_metadata, recorder_config) where each element is the
            parsed YAML content (typically a dict).
    """
    experiment_parameters_path = recording_dir / "metadata/experiment_parameters.yaml"
    recorder_config_path = recording_dir / "metadata/recorder_config.yaml"
    with open(experiment_parameters_path, "r") as f:
        experiment_metadata = yaml.safe_load(f)
    with open(recorder_config_path, "r") as f:
        recorder_config = yaml.safe_load(f)
    return experiment_metadata, recorder_config


def _determine_adaptive_muscle_vrange(
    muscle_vrange_quantiles,
    muscle_vrange_quantiles_sample_rate,
    muscle_image_paths,
    vmin_percentile_of_quantiles=50.0,
    vmax_percentile_of_quantiles=95.0,
):
    """Determine an adaptive (vmin, vmax) for muscle image visualization.

    The function samples a subset of the provided muscle images, computes the
    requested quantiles for each sampled image, then computes robust percentiles
    across those per-image quantiles to produce final vmin and vmax values.

    Args:
        muscle_vrange_quantiles (tuple[float, float]): Percentiles to compute on
            each sampled image (e.g. (97.0, 99.995)). Must not be None.
        muscle_vrange_quantiles_sample_rate (float): Fraction of images to
            sample when determining the range (0 < sample_rate <= 1).
        muscle_image_paths (Sequence[Path]): List of file paths to muscle images
            (tif files) to sample from.
        vmin_percentile_of_quantiles (float): Percentile to take across the
            per-image lower-quantiles to produce the final vmin (default 50.0).
        vmax_percentile_of_quantiles (float): Percentile to take across the
            per-image upper-quantiles to produce the final vmax (default 95.0).

    Returns:
        tuple[int, int]: (vmin, vmax) integer values suitable for clipping and
        mapping muscle images to display range.

    Raises:
        ValueError: If ``muscle_vrange_quantiles`` is None or if the sample rate
            is invalid.
    """
    if muscle_vrange_quantiles is None:
        logging.error(
            "User must specify either muscle_vrange or muscle_vrange_quantiles."
        )
        raise ValueError("Unspecified muscle_vrange or muscle_vrange_quantiles.")

    if not (0 < muscle_vrange_quantiles_sample_rate <= 1.0):
        raise ValueError("muscle_vrange_quantiles_sample_rate must be in (0, 1].")

    sample_every_k = max(1, int(1 / muscle_vrange_quantiles_sample_rate))
    paths_to_check = muscle_image_paths[::sample_every_k]

    quantiles = np.zeros((len(paths_to_check), 2))
    print("Detecting muscle vrange adaptively...")
    for i, path in tqdm(
        enumerate(paths_to_check),
        desc="Determining muscle vrange",
        total=len(paths_to_check),
        disable=None,
    ):
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        quantiles[i, :] = np.percentile(image.flat, muscle_vrange_quantiles)
    vmin = np.percentile(quantiles[:, 0], vmin_percentile_of_quantiles)
    vmax = np.percentile(quantiles[:, 1], vmax_percentile_of_quantiles)

    return int(vmin), int(vmax)


def _calculate_num_usable_images(
    num_behavior_images_total: int,
    num_muscle_images_total: int,
    behavior_muscle_sync_ratio: int,
    num_muscle_images_discrepancy_threshold: int = 1,
    num_behavior_images_discrepancy_threshold: int = 3,
) -> tuple[int, int]:
    """Calculate the number of usable behavior and muscle images to ensure consistency
    between the two recordings. This function accounts for potential discrepancies
    caused by asynchronous recording threads.

    Args:
        num_behavior_images_total (int): Total number of behavior images recorded.
        num_muscle_images_total (int): Total number of muscle images recorded.
        behavior_muscle_sync_ratio (int): The synchronization ratio between behavior
            and muscle recordings.
        num_muscle_images_discrepancy_threshold (int, optional): The allowed
            discrepancy  threshold for muscle images. Defaults to 1.
        num_behavior_images_discrepancy_threshold (int, optional): The allowed
            discrepancy  threshold for behavior images. Defaults to 3.

    Returns:
        tuple: A tuple containing:
            - num_behavior_images_usable (int): The number of usable behavior images.
            - num_muscle_images_usable (int): The number of usable muscle images.

    Logs:
        Warnings are logged if the discrepancy between the calculated usable images
        and the total recorded images exceeds the allowed thresholds.
    """
    num_muscle_images_usable = min(
        num_muscle_images_total,
        num_behavior_images_total // behavior_muscle_sync_ratio,
    )

    if (
        num_muscle_images_total - num_muscle_images_usable
        > num_muscle_images_discrepancy_threshold
    ):
        logging.warning(
            f"The number of behavior images ({num_behavior_images_total}) divided by "
            f"the behavior:muscle sync ratio ({behavior_muscle_sync_ratio}) results in "
            f"{num_muscle_images_usable} muscle images to be used. However, "
            f"{num_muscle_images_total} muscle images are recorded. This discrepancy "
            f"is greater than the allowed threshold"
            f"{num_muscle_images_discrepancy_threshold}. This indicates that the "
            f"muscle and behavior recordings are not in sync."
        )

    num_behavior_images_usable = num_muscle_images_usable * behavior_muscle_sync_ratio
    if (
        num_behavior_images_total - num_behavior_images_usable
        > num_behavior_images_discrepancy_threshold
    ):
        logging.warning(
            f"The number of muscle images ({num_muscle_images_total}) multiplied by "
            f"the behavior:muscle sync ratio ({behavior_muscle_sync_ratio}) results in "
            f"{num_behavior_images_usable} behavior images to be used. However, "
            f"{num_behavior_images_total} behavior images are recorded. This "
            f"discrepancy is greater than the allowed threshold"
            f"{num_behavior_images_discrepancy_threshold}. This indicates that the "
            f"muscle and behavior recordings are not in sync."
        )

    return num_behavior_images_usable, num_muscle_images_usable


def generate_overlay_samples(
    recording_dir: Path,
    muscle_vrange: tuple[int, int] | None = None,
    muscle_vrange_quantiles: tuple[float, float] | None = (97.0, 99.995),
    muscle_vrange_quantiles_sample_rate: float = 0.05,
    num_samples: int = 100,
    num_samples_per_row: int = 10,
    panel_size: tuple[int, int] = (4, 5),  # width, height per matplotlib style
    overwrite: bool = False,
) -> None:
    """Create a grid of overlay sample images combining behavior and muscle frames.

    The resulting image is saved to ``<recording_dir>/processed/overlay_samples.jpg``.
    Each cell shows the behavior (grayscale) on the red channel and the muscle
    activity mapped to the green channel.

    Args:
        recording_dir (Path): Path to the recording directory (expects a
            ``processed`` subdirectory with `muscle_frames_metadata.csv` and
            processed image files).
        muscle_vrange (tuple[int, int] | None): Optional (vmin, vmax) used to
            normalize muscle images for display. If None, an adaptive range is
            computed using ``muscle_vrange_quantiles``.
        muscle_vrange_quantiles (tuple[float, float] | None): Percentiles used
            to determine the adaptive muscle dynamic range when ``muscle_vrange``
            is None. Expressed as percentages (e.g. ``(97.0, 99.995)``).
        muscle_vrange_quantiles_sample_rate (float): Fraction of muscle images to
            sample when computing the adaptive range (e.g. 0.05 samples 5% of
            images).
        num_samples (int): Number of sample frames to include in the grid.
        num_samples_per_row (int): Number of columns in the output grid.
        panel_size (tuple[int, int]): Size (width, height) per sample panel in
            inches (matplotlib style).
        overwrite (bool): If False and the output image already exists the
            function raises a RuntimeError; if True it will overwrite.

    Returns:
        None: The composed image is written to disk and nothing is returned.
    """
    processed_dir = recording_dir / "processed"
    muscle_metadata_path = processed_dir / "muscle_frames_metadata.csv"

    # Check output
    output_image_path = processed_dir / "overlay_samples.jpg"
    if output_image_path.is_file() and not overwrite:
        logging.error(
            f"Output image {output_image_path} already exists. Change the output path "
            f"or use `overwrite=True`."
        )
        raise RuntimeError("Output image already exists.")

    # Load muscle frames metadata
    muscle_metadata_df = pd.read_csv(muscle_metadata_path)
    sample_interval = len(muscle_metadata_df) // num_samples
    sample_muscle_frame_ids = np.arange(0, len(muscle_metadata_df), sample_interval)
    sample_muscle_frame_ids = np.unique(sample_muscle_frame_ids)
    num_samples = len(sample_muscle_frame_ids)

    # If necessary, determine vmin and vmax of muscle images for visualization
    if muscle_vrange is None:
        unwarped_muscle_images_paths = sorted(
            list(recording_dir.glob("muscle_images/*.tif"))
        )
        muscle_vrange = _determine_adaptive_muscle_vrange(
            muscle_vrange_quantiles,
            muscle_vrange_quantiles_sample_rate,
            unwarped_muscle_images_paths,
        )
        print(f"Determined adaptive muscle value range: {muscle_vrange}.")

    # Set up figure
    num_rows = (num_samples + num_samples_per_row - 1) // num_samples_per_row
    fig, axes = plt.subplots(
        num_rows,
        num_samples_per_row,
        figsize=(panel_size[0] * num_samples_per_row, panel_size[1] * num_rows),
        tight_layout=True,
    )
    axes = axes.flatten()

    # Generate samples
    print("Generating samples overlay images...")
    for i, muscle_frame_id in tqdm(
        enumerate(sample_muscle_frame_ids),
        total=num_samples,
        desc="Generating overlay samples",
        disable=None,
    ):
        metadata_entry = muscle_metadata_df.iloc[muscle_frame_id]
        assert metadata_entry["muscle_frame_id"] == muscle_frame_id
        behavior_frame_id = metadata_entry["corresponding_behavior_frame_id"]
        muscle_image = load_processed_muscle_image(recording_dir, muscle_frame_id)
        behavior_image = load_processed_behavior_frame(recording_dir, behavior_frame_id)
        if behavior_image is None:  # behavior id must be out of bounds
            i -= 1
            break

        # Overlay images
        width = min(muscle_image.shape[1], behavior_image.shape[1])
        muscle_image = muscle_image[:, :width]
        overlay = np.zeros((behavior_image.shape[0], width, 3), dtype=np.uint8)
        overlay[:, :, 0] = behavior_image[:, :width]
        muscle_image_normalized = (
            np.clip(muscle_image.astype(np.float32), muscle_vrange[0], muscle_vrange[1])
            - muscle_vrange[0]
        ) / (muscle_vrange[1] - muscle_vrange[0])
        overlay[:, :, 1] = (255 * muscle_image_normalized).astype(np.uint8)
        axes[i].imshow(overlay)
        axes[i].set_title(
            f"Behavior frame {behavior_frame_id}, muscle frame {muscle_frame_id}"
        )

    for j in range(i + 1, len(axes)):
        axes[j].axis("off")

    fig.savefig(output_image_path)
