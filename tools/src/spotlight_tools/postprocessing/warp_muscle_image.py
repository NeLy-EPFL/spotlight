import logging
import cv2
import numpy as np
import pandas as pd
import yaml
from joblib import Parallel, delayed
from pathlib import Path
from tqdm import tqdm

from spotlight_tools.calibration import (
    SpotlightPositionMapper,
    BehaviorMuscleCrossMapper,
)


_imwrite_compression_params = [cv2.IMWRITE_TIFF_COMPRESSION, 5]
# TIFF compression methods:
#   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
#   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
#   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
#   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
#   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html
# cv::IMWRITE_TIFF_COMPRESSION_LZW is used by the recording GUI


def process_muscle_data(
    recording_dir: Path,
    num_frames: int | None = None,
    overwrite: bool = False,
    missing_muscle_frames_tolerance: int = 3,
    num_workers: int = -1,
) -> None:
    """Process muscle images to align them with behavior frames.

    Args:
        recording_dir: Path to the recording directory (as set in the
            Spotlight recorder GUI).
        num_frames: If specified, only process this many frames
            (for debugging).
        overwrite: If True, overwrite existing processed muscle images.
        missing_muscle_frames_tolerance: Number of missing muscle frames
            tolerated at the end of the recording.
        num_workers: Number of parallel workers to use for processing
            muscle images. If -1, use all available cores.

    Returns:
        muscle_frame_metadata (pd.DataFrame)
    """
    # IO checks
    raw_muscle_images_dir = recording_dir / "muscle_images"
    processed_dir = recording_dir / "processed"
    processed_muscle_images_dir = processed_dir / "muscle_images"
    interpolated_stage_pos_path = processed_dir / "behavior_frames_metadata.csv"
    if processed_muscle_images_dir.is_dir() and not overwrite:
        logging.error(
            f"Output directory (processed muscle images) directory "
            f"'{processed_muscle_images_dir}' already exists. "
            f"Remove it or set overwrite to True."
        )
        return
    processed_muscle_images_dir.mkdir(exist_ok=True, parents=True)

    # Load the stage positions at muscle recording frames
    timing_metadata_path = recording_dir / "metadata/dual_recording_timing.yaml"
    with open(timing_metadata_path, "r") as f:
        timing_metadata = yaml.safe_load(f)
    muscle_behavior_sync_ratio = timing_metadata["sync_ratio"]
    stage_pos_df_at_behavior_frames = pd.read_csv(interpolated_stage_pos_path)
    stage_pos_df_at_muscle_frames = stage_pos_df_at_behavior_frames[
        muscle_behavior_sync_ratio::muscle_behavior_sync_ratio
    ]

    # Load the recorder config file to get the behavior image dimensions
    recorder_config_path = recording_dir / "metadata/recorder_config.yaml"
    with open(recorder_config_path, "r") as f:
        recorder_config = yaml.safe_load(f)
    # * Attention! The recorder config file specifies ROI dimensions as
    # defined on the physical sensor. The acquisition software flips the
    # image and rotates it by 90 degrees in order to keep it consistent
    # with the fly arena orientation. Here the dimension should be the
    # dimension of the reoriented image. Therefore, nrow is roi_width
    # from the recorder config and ncol is roi_height.
    behavior_image_dim = (
        recorder_config["behavior_camera"]["roi_width"],  # actually height
        recorder_config["behavior_camera"]["roi_height"],  # actually width
    )

    # Create mapping object
    behavior_calibration_path = (
        recording_dir / "metadata/calibration_parameters_behavior.yaml"
    )
    muscle_calibration_path = (
        recording_dir / "metadata/calibration_parameters_muscle.yaml"
    )
    behavior_mapper = SpotlightPositionMapper(behavior_calibration_path)
    muscle_mapper = SpotlightPositionMapper(muscle_calibration_path)
    mapper = BehaviorMuscleCrossMapper(behavior_mapper, muscle_mapper)

    # Check if we have all the muscle images
    _muscle_paths_by_frame_idx = {}
    for path in raw_muscle_images_dir.glob("*.tif"):
        try:
            frame_idx = int(path.stem.split("_")[-1])
        except ValueError:
            logging.warning(
                f"Problem scanning muscle images: Could not parse frame index from "
                f"file name {path.name}. Skipping this file."
            )
            continue
        _muscle_paths_by_frame_idx[frame_idx] = path

    muscle_image_paths = []
    num_expected_frames = stage_pos_df_at_muscle_frames.shape[0]
    for frame_idx in range(num_expected_frames):
        if frame_idx not in _muscle_paths_by_frame_idx:
            if frame_idx >= num_expected_frames - missing_muscle_frames_tolerance:
                # If we are almost at the end of the recording, it's ok. This could
                # simply be due to expected synchronization/timing imperfections.
                break
            logging.error(
                f"Problem scanning muscle images: Frame {frame_idx} not found in "
                f"{raw_muscle_images_dir} (a total of {num_expected_frames} is "
                f"expected). Dataset is incomplete."
            )
            raise RuntimeError("Dataset is incomplete.")
        muscle_image_paths.append(_muscle_paths_by_frame_idx[frame_idx])
    num_muscle_frames = len(muscle_image_paths)
    if num_muscle_frames != num_expected_frames:
        logging.warning(
            f"Found {num_muscle_frames} muscle images, but expected "
            f"{num_expected_frames}. This is likely normal because the two cameras "
            f"receive the stop signal at slightly different times."
        )
        stage_pos_df_at_muscle_frames = stage_pos_df_at_muscle_frames.iloc[
            :num_muscle_frames
        ].reset_index(drop=True)

    if num_frames is not None:
        muscle_image_paths = muscle_image_paths[:num_frames]
        stage_pos_df_at_muscle_frames = stage_pos_df_at_muscle_frames[:num_frames]

    # Create muscle frame metadata dataframe
    # (acquired and received times to be filled later)
    muscle_frame_metadata = pd.DataFrame(
        {
            "muscle_frame_id": np.arange(len(muscle_image_paths)),
            "corresponding_behavior_frame_id": stage_pos_df_at_muscle_frames[
                "behavior_frame_id"
            ].values,
            "acquired_time_us": np.full(len(muscle_image_paths), -1, dtype=np.int64),
            "received_time_us": np.full(len(muscle_image_paths), -1, dtype=np.int64),
            "x_pos_mm_interp": stage_pos_df_at_muscle_frames["x_pos_mm_interp"].values,
            "y_pos_mm_interp": stage_pos_df_at_muscle_frames["y_pos_mm_interp"].values,
        }
    )

    # Process muscle images in parallel
    payload_kwargs = []
    for i, muscle_image_path in enumerate(muscle_image_paths):
        stage_pos = stage_pos_df_at_muscle_frames.iloc[i][
            ["x_pos_mm_interp", "y_pos_mm_interp"]
        ].values.astype(np.float32)
        output_path = processed_muscle_images_dir / muscle_image_path.name
        payload_kwargs.append(
            {
                "mapper": mapper,
                "input_path": muscle_image_path,
                "stage_pos": stage_pos,
                "behavior_image_dim": behavior_image_dim,
                "output_path": output_path,
            }
        )
    parallel_runner = Parallel(n_jobs=num_workers)
    print(
        f"Warping {len(payload_kwargs)} muscle images using {num_workers} workers"
        f" (effectively {parallel_runner._effective_n_jobs} workers)"
    )
    parallel_runner(
        delayed(_process_single_frame)(**kwargs)
        for kwargs in tqdm(
            payload_kwargs, desc="Warping muscle images", total=len(payload_kwargs)
        )
    )

    # Merge metadata into a single file
    print("Merging metadata")
    for muscle_path in muscle_image_paths:
        metadata_path = str(muscle_path).replace(".tif", ".csv")
        metadata_this_frame = pd.read_csv(metadata_path).iloc[0]
        acquired_time_us = metadata_this_frame["acquired_time_us"]
        received_time_us = metadata_this_frame["received_time_us"]
        muscle_frame_metadata.loc[i, "acquired_time_us"] = acquired_time_us
        muscle_frame_metadata.loc[i, "received_time_us"] = received_time_us

    # Save metadata
    int_columns = [
        "muscle_frame_id",
        "corresponding_behavior_frame_id",
        "acquired_time_us",
        "received_time_us",
    ]
    muscle_frame_metadata[int_columns] = muscle_frame_metadata[int_columns].astype(
        np.int64
    )
    metadata_output_path = processed_dir / "muscle_frames_metadata.csv"
    muscle_frame_metadata.to_csv(metadata_output_path, index=False)
    print(
        f"Processed muscle images saved to {processed_muscle_images_dir}. "
        f"Metadata saved to {metadata_output_path}."
    )
    return muscle_frame_metadata


def _process_single_frame(
    mapper: BehaviorMuscleCrossMapper,
    input_path: Path,
    stage_pos: np.ndarray,
    behavior_image_dim: tuple[int, int],
    output_path: Path,
):
    in_image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    out_image = mapper.transform_image_muscle2behavior(
        stage_pos, in_image, behavior_image_dim
    )
    cv2.imwrite(str(output_path), out_image, _imwrite_compression_params)
