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


def visualize_stage_trajectory(
    consolidated_metadata_df: pd.DataFrame,
) -> tuple[Figure, Axes]:
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
    output_video_path: Path | None = None,
    muscle_vrange: tuple[int, int] | None = None,
    muscle_vrange_quantiles: tuple[float, float] | None = (97.0, 99.995),
    muscle_vrange_quantiles_sample_rate: float = 0.05,
    play_fps: int = 30,
    num_frames: int | None = None,
    overwrite: bool = False,
) -> None:
    """
    Generates a summary video by combining behavior and muscle images from a recording.
    Args:
        recording_dir (Path): Path to the directory containing the recording data.
        output_video_path (Path | None, optional): Path to save the generated summary
            video. If None, the video will be saved as "summary_video.mp4" in the
            "processed" subdirectory of `recording_dir`. Defaults to None.
        muscle_vrange (tuple[int, int] | None, optional): Value range for normalizing
            muscle images. If None, the range will be determined adaptively based on
            quantiles. Defaults to None.
        muscle_vrange_quantiles (tuple[float, float] | None, optional): Quantiles to use
            for determining the adaptive value range of muscle images (if muscle_vrange
            is not provided). Defaults to (97.0, 99.995).
        muscle_vrange_quantiles_sample_rate (float, optional): Sampling rate for
            determining the adaptive value range of muscle images. Only this portion of
            all muscle images are scanned to adaptively determine the vrange. Defaults
            to 0.05.
        play_fps (int, optional): FPS for the output video. Defaults to 30.
        num_frames (int | None, optional): Maximum number of frames to include in the
            video. Can be useful for testing purposes. If set, the video will contain
            only the first `num_frames` frames. If None, all frames will be processed.
        overwrite (bool, optional): Whether to overwrite the output video if it already
            exists. Defaults to False.
    Raises:
        RuntimeError: If the behavior video cannot be opened.
    Notes:
        - The function assumes that the recording directory contains preprocessed data,
          including a behavior video and muscle images.
        - The behavior-to-muscle images are synchronized based on the
          `muscle_sync_ratio` from the experiment metadata.
        - The output video consists of concatenated behavior and muscle images, with the
          behavior image in grayscale and the muscle image in green.
    """
    _check_if_preprocessed(recording_dir)
    experiment_metadata, recorder_config = _load_metadata(recording_dir)
    processed_dir = recording_dir / "processed"
    behavior_video_path = processed_dir / "behavior_video.mkv"
    muscle_images_dir = processed_dir / "muscle_images"
    sync_ratio = experiment_metadata["muscle_sync_ratio"]

    if output_video_path is None:
        output_video_path = processed_dir / "summary_video.mp4"
    if output_video_path.is_file() and not overwrite:
        logging.error(
            f"Output video {output_video_path} already exists. Change the output path "
            f"or use `overwrite=True`."
        )
        raise RuntimeError("Output video already exists.")

    # Index files to be used
    # behavior_images_paths = sorted(list(behavior_images_dir.glob("*.jpg")))
    width, height, num_behavior_images = _get_video_info(behavior_video_path)
    muscle_images_paths = sorted(list(muscle_images_dir.glob("*.tif")))
    num_behavior_images_usable, num_muscle_images_usable = _calculate_num_usable_images(
        num_behavior_images, len(muscle_images_paths), sync_ratio
    )
    if num_frames is not None:
        num_behavior_images_usable = min(num_behavior_images_usable, num_frames)
        num_muscle_images_usable = num_behavior_images_usable // sync_ratio
    muscle_images_paths = muscle_images_paths[:num_muscle_images_usable]

    # Check if nominal width of image is incorrect
    _muscle_image_sample = cv2.imread(str(muscle_images_paths[0]), cv2.IMREAD_UNCHANGED)
    assert len(_muscle_image_sample.shape) == 2, "Muscle image is not single channel"
    muscle_width = _muscle_image_sample.shape[1]
    if muscle_width != width:
        _behavior_image_width = width
        width = min(width, muscle_width)
        logging.warning(
            f"Behavior images have a width of {_behavior_image_width} pixels, but "
            f"muscle images have a width of {muscle_width} pixels. This is likely "
            f"because the user set the desired ROI height is not allowed  by the "
            f"camera, so the camera rounded it to the nearest allowed value. (Note: "
            f"the canonically oriented behavior images is 90-degree rotated and "
            f"flipped compared to how the camera sensor acquires them; hence it's "
            f"the ROI height parameter that matters. Taking the lower of the widths."
        )

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

    # Initialize video writer
    writer = cv2.VideoWriter(
        str(output_video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        play_fps,
        (width * 2, height),
        True,
    )

    # Initialize behavior image reader
    behavior_video_reader = cv2.VideoCapture(str(behavior_video_path))
    if not behavior_video_reader.isOpened():
        logging.error(f"Error: Could not open behavior video {behavior_video_path}.")
        raise RuntimeError("Could not open behavior video.")

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
        behavior_image = behavior_image[:, :width, 0]

        # Muscle image
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

        # Concatenate images
        concatenated = np.zeros((height, width * 2, 3), dtype=np.uint8)
        concatenated[:, :width, :] = behavior_image[:, :, np.newaxis]
        concatenated[:, width : 2 * width, 1] = muscle_image

        writer.write(concatenated)

    writer.release()


def _get_video_info(video_path: Path):
    video = cv2.VideoCapture(str(video_path))
    if not video.isOpened():
        print("Error: Could not open video.")
        return None

    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    video.release()

    return width, height, frame_count


def _load_metadata(recording_dir: Path):
    experiment_parameters_path = recording_dir / "metadata/experiment_parameters.yaml"
    recorder_config_path = recording_dir / "metadata/recorder_config.yaml"
    with open(experiment_parameters_path, "r") as f:
        experiment_metadata = yaml.safe_load(f)
    with open(recorder_config_path, "r") as f:
        recorder_config = yaml.safe_load(f)
    return experiment_metadata, recorder_config


def _check_if_preprocessed(processed_dir: Path):
    processed_dir = processed_dir / "processed"
    muscle_images_dir = processed_dir / "muscle_images"
    behavior_frame_metadata_path = processed_dir / "behavior_frames_metadata.csv"
    if not muscle_images_dir.is_dir() or not behavior_frame_metadata_path.is_file():
        logging.critical(
            f"Data has not been preprocessed. Please run the preprocessing "
            f"script before generating summary videos."
        )
        raise RuntimeError("Data has not been preprocessed.")


def _determine_adaptive_muscle_vrange(
    muscle_vrange_quantiles,
    muscle_vrange_quantiles_sample_rate,
    muscle_image_paths,
    vmin_percentile_of_quantiles=50.0,
    vmax_percentile_of_quantiles=95.0,
):
    if muscle_vrange_quantiles is None:
        logging.error(
            "User must specify either muscle_vrange or muscle_vrange_quantiles."
        )
        raise ValueError("Unspecified muscle_vrange or muscle_vrange_quantiles.")

    sample_every_k = int(1 / muscle_vrange_quantiles_sample_rate)
    paths_to_check = muscle_image_paths[::sample_every_k]

    quantiles = np.zeros((len(paths_to_check), 2))
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


def load_muscle_image(recording_dir, muscle_frame_id):
    muscle_image_path = (
        recording_dir
        / "processed/muscle_images"
        / f"muscle_frame_{muscle_frame_id:09d}.tif"
    )
    return cv2.imread(str(muscle_image_path), cv2.IMREAD_UNCHANGED)


def load_behavior_frame(recording_dir, behavior_frame_id):
    behavior_video_path = recording_dir / "processed/behavior_video.mkv"
    behavior_video_capture = cv2.VideoCapture(str(behavior_video_path))
    num_frames = int(behavior_video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if behavior_frame_id >= num_frames:
        return None  # Return None if the frame ID is out of bounds
    behavior_video_capture.set(cv2.CAP_PROP_POS_FRAMES, behavior_frame_id)
    ret, frame = behavior_video_capture.read()
    if not ret:
        raise ValueError(f"Could not read frame {behavior_frame_id} from video.")
    behavior_video_capture.release()
    return frame[:, :, 0]


def generate_overlay_samples(
    recording_dir: Path,
    output_image_path: Path | None = None,
    muscle_vrange: tuple[int, int] | None = None,
    muscle_vrange_quantiles: tuple[float, float] | None = (97.0, 99.995),
    muscle_vrange_quantiles_sample_rate: float = 0.05,
    num_samples: int = 100,
    num_samples_per_row: int = 10,
    panel_size: tuple[int, int] = (4, 5),  # width, height per matplotlib style
    overwrite: bool = False,
) -> None:
    """Generate overlay samples from muscle and behavior images.

    Args:
        recording_dir (Path): Path to the directory containing the recording data.
        output_image_path (Path | None, optional): Path to save the generated overlay
            samples image. If None, the image will be saved as "overlay_samples.png" in
            the "processed" subdirectory of `recording_dir`. Defaults to None.
        muscle_vrange (tuple[int, int] | None, optional): Value range for normalizing
            muscle images. If None, the range will be determined adaptively based on
            quantiles. Defaults to None.
        muscle_vrange_quantiles (tuple[float, float] | None, optional): Quantiles to use
            for determining the adaptive value range of muscle images (if muscle_vrange
            is not provided). Defaults to (97.0, 99.995).
        muscle_vrange_quantiles_sample_rate (float, optional): Sampling rate for
            determining the adaptive value range of muscle images. Only this portion of
            all muscle images are scanned to adaptively determine the vrange. Defaults
            to 0.05.
        num_samples (int, optional): Number of samples to generate. Defaults to 100.
        num_samples_per_row (int, optional): Number of samples to display per row in
            the output image. Defaults to 10.
        panel_size (tuple[int, int], optional): Size of each panel in the output image
            in matplotlib style (width, height). Defaults to (4, 5).
        overwrite (bool, optional): Whether to overwrite the output image if it already
            exists. Defaults to False.
    """
    _check_if_preprocessed(recording_dir)
    processed_dir = recording_dir / "processed"
    muscle_metadata_path = processed_dir / "muscle_frames_metadata.csv"

    # Check output
    if output_image_path is None:
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
    for i, muscle_frame_id in tqdm(
        enumerate(sample_muscle_frame_ids),
        total=num_samples,
        desc="Generating overlay samples",
    ):
        metadata_entry = muscle_metadata_df.iloc[muscle_frame_id]
        assert metadata_entry["muscle_frame_id"] == muscle_frame_id
        behavior_frame_id = metadata_entry["corresponding_behavior_frame_id"]
        muscle_image = load_muscle_image(recording_dir, muscle_frame_id)
        behavior_image = load_behavior_frame(recording_dir, behavior_frame_id)
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
