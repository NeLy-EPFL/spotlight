import logging
import cv2
import numpy as np
import pandas as pd
import yaml
import h5py
from joblib import Parallel, delayed
from pathlib import Path

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


def map_muscle_frames_to_behavior(
    *,
    muscle_calib_path: Path,
    behavior_calib_path: Path,
    dual_recording_timing_path: Path,
    processed_behavior_frame_metadata_path: Path,
    behavior_alignment_metadata_path: Path,
    raw_muscle_images_dir: Path,
    transformed_muscle_images_output_dir: Path,
    muscle_metadata_output_path: Path,
    missing_muscle_frames_tolerance: int = 3,
    num_workers: int = -1,
):
    """This function performs muscle-to-behavior frame mapping and transformation:

    1. Determines timing synchronization between muscle and behavior recordings.
    2. Spatially maps muscle images to behavior coordinate system using Spotlight
       calibration parameters.
    3. Applies the same alignment transformations used for behavior frames.
    4. Saves transformed muscle images in TIFF format and metadata.

    Args:
        muscle_calib_path (Path): Path to muscle camera calibration parameters.
        behavior_calib_path (Path): Path to behavior camera calibration parameters.
        dual_recording_timing_path (Path): Path to dual recording timing metadata.
        processed_behavior_frame_metadata_path (Path): Path to behavior frames metadata
            (CSV).
        behavior_alignment_metadata_path (Path): Path to behavior alignment transforms
            (HDF5).
        raw_muscle_images_dir (Path): Directory containing raw muscle image files.
        transformed_muscle_images_output_dir (Path): Directory to save output frames.
        muscle_metadata_output_path (Path): Output path for muscle frames metadata (CSV).
        missing_muscle_frames_tolerance (int): See
            `scripts.postprocess_recording.postprocess_recording_data`.
        num_workers (int): Number of parallel workers (-1 for all available cores).

    Returns:
        None: Outputs are saved to the specified directories and files.
    """
    logger = logging.getLogger(__name__)

    # Get behavior-muscle sync ratio
    behavior_muscle_sync_ratio = get_behavior_muscle_sync_ratio(
        dual_recording_timing_path
    )

    # Load the stage positions at muscle recording frames
    stage_pos_df_at_muscle_frames = _get_stage_pos_df_at_muscle_frames(
        processed_behavior_frame_metadata_path, behavior_muscle_sync_ratio
    )

    # Create mapping object
    behavior_mapper = SpotlightPositionMapper(behavior_calib_path)
    muscle_mapper = SpotlightPositionMapper(muscle_calib_path)
    cross_mapper = BehaviorMuscleCrossMapper(behavior_mapper, muscle_mapper)

    # Check if we have all the muscle images
    muscle_image_paths = _filter_muscle_frames_by_availability(
        raw_muscle_images_dir,
        stage_pos_df_at_muscle_frames,
        missing_muscle_frames_tolerance,
    )
    stage_pos_df_at_muscle_frames = stage_pos_df_at_muscle_frames.iloc[
        : len(muscle_image_paths)
    ].reset_index(drop=True)

    # Load transformation matrices applied to behavior frames (for alignment)
    alignment_transforms, output_dim = _load_alignment_transform_metadata(
        behavior_alignment_metadata_path, behavior_muscle_sync_ratio
    )

    # Prepare input kwargs for parallel processing
    input_kwargs = []
    for i, input_path in enumerate(muscle_image_paths):
        stage_pos = stage_pos_df_at_muscle_frames.iloc[i][
            ["x_pos_mm_interp", "y_pos_mm_interp"]
        ].values.astype(np.float32)
        muscle2behavior_transform_matrix = (
            cross_mapper.get_affine_matrix_muscle2behavior(stage_pos)
        )
        behavior_alignment_transform_matrix = alignment_transforms[i]
        output_path = transformed_muscle_images_output_dir / input_path.name
        kwargs = {
            "muscle2behavior_trans_mat": muscle2behavior_transform_matrix,
            "behavior_alignment_trans_mat": behavior_alignment_transform_matrix,
            "input_path": input_path,
            "output_dim": output_dim,
            "output_path": output_path,
            "return_output": False,  # reduce IO stress
        }
        input_kwargs.append(kwargs)

    # Process muscle images in parallel
    transformed_muscle_images_output_dir.mkdir(parents=True, exist_ok=True)
    parallel_runner = Parallel(n_jobs=num_workers, backend="loky")
    logger.info(
        f"Warping {len(input_kwargs)} muscle images using {num_workers} workers"
        f" (effectively {parallel_runner._effective_n_jobs()} workers)"
    )
    parallel_runner(
        delayed(apply_affine_transform_to_muscle_image)(**kwargs)
        for kwargs in input_kwargs
    )
    logger.info(
        "Finished warping muscle images; "
        f"outputs saved to {transformed_muscle_images_output_dir}"
    )

    # Save muscle metadata as a dataframe
    muscle_frame_metadata = _make_muscle_metadata_dataframe(
        muscle_image_paths, stage_pos_df_at_muscle_frames
    )
    muscle_metadata_output_path.parent.mkdir(parents=True, exist_ok=True)
    muscle_frame_metadata.to_csv(muscle_metadata_output_path, index=False)
    logger.info(f"Muscle frame metadata saved to {muscle_metadata_output_path}")


def get_behavior_muscle_sync_ratio(dual_recording_timing_path: Path) -> int:
    """Extract behavior-to-muscle frame synchronization ratio from timing metadata."""
    with open(dual_recording_timing_path, "r") as f:
        timing_metadata = yaml.safe_load(f)
    return timing_metadata["sync_ratio"]


def _get_stage_pos_df_at_muscle_frames(
    interpolated_stage_pos_path: Path,
    behavior_muscle_sync_ratio: int,
) -> pd.DataFrame:
    stage_pos_df_at_behavior_frames = pd.read_csv(interpolated_stage_pos_path)
    stage_pos_df_at_muscle_frames = stage_pos_df_at_behavior_frames[
        behavior_muscle_sync_ratio::behavior_muscle_sync_ratio
    ]
    return stage_pos_df_at_muscle_frames


def _filter_muscle_frames_by_availability(
    raw_muscle_images_dir: Path,
    stage_pos_df_at_muscle_frames: pd.DataFrame,
    missing_muscle_frames_tolerance: int = 3,
) -> list[Path]:
    # Index all available frames
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

    # Check if each expected frame is among the frames found
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
        else:
            muscle_image_paths.append(_muscle_paths_by_frame_idx[frame_idx])

    num_muscle_frames = len(muscle_image_paths)
    if num_muscle_frames != num_expected_frames:
        logging.warning(
            f"Found {num_muscle_frames} muscle images, but expected "
            f"{num_expected_frames}. This is likely normal because the two cameras "
            f"receive the stop signal at slightly different times."
        )

    return muscle_image_paths


def _load_alignment_transform_metadata(
    behavior_alignment_metadata_path, behavior_muscle_sync_ratio
):
    with h5py.File(behavior_alignment_metadata_path, "r") as f:
        transforms_ds = f["transform_matrices"]
        alignment_transforms = transforms_ds[
            behavior_muscle_sync_ratio::behavior_muscle_sync_ratio, :, :
        ]
        output_dim = transforms_ds.attrs["output_dim"]
    return alignment_transforms, output_dim


def apply_affine_transform_to_muscle_image(
    muscle2behavior_trans_mat: np.ndarray,
    behavior_alignment_trans_mat: np.ndarray,
    input_path: Path,
    output_dim: tuple[int, int],
    output_path: Path,
    return_output: bool = True,
):
    """Apply composed affine transformation to align a single muscle frame with the
    corresponding behavior frame.

    Args:
        muscle2behavior_trans_mat (np.ndarray): 2x3 transformation matrix mapping muscle
            to behavior coordinates (derived from Spotlight calibration).
        behavior_alignment_trans_mat (np.ndarray): 2x3 transformation matrix for
            behavior frame alignment (whatever that's been applied to the behavior
            frame; this can be read out from behavior alignment transform metadata).
        input_path (Path): Path to the input muscle image file.
        output_dim (tuple[int, int]): Output image dimensions (width, height).
        output_path (Path): Path where the transformed image will be saved.
        return_output (bool): Whether to return the transformed image array. Not
            returning anything may help reduce IO load if called in parallel (automatic
            garbage collection might not happen until after the map operation).

    Returns:
        np.ndarray or None (depending on return_output): Transformed muscle frame.
    """
    # Convert 2x3 transform matrices to 3x3 homogeneous matrices for easier composition
    muscle2behavior_trans_mat = np.vstack([muscle2behavior_trans_mat, [0, 0, 1]])
    behavior_alignment_trans_mat = np.vstack([behavior_alignment_trans_mat, [0, 0, 1]])

    # Compose transforms: first muscle->behavior, then apply the same alignment
    # transform that was applied to the behavior frame.
    composed_trans_mat = behavior_alignment_trans_mat @ muscle2behavior_trans_mat
    assert np.allclose(composed_trans_mat[2, :], [0, 0, 1])

    # Convert back to 2x3 for cv2.warpAffine
    composed_trans_mat = composed_trans_mat[:2, :]

    # Load muscle image and apply transform
    in_image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    out_image = cv2.warpAffine(in_image, composed_trans_mat, output_dim)

    # Save output image
    cv2.imwrite(str(output_path), out_image, _imwrite_compression_params)

    if return_output:
        return out_image
    else:
        # don't return anything - might help reduce IO traffic if called in parallel
        return None


def _make_muscle_metadata_dataframe(muscle_image_paths, stage_pos_df_at_muscle_frames):
    muscle_frame_ids = np.arange(len(muscle_image_paths))
    behavior_frame_ids = stage_pos_df_at_muscle_frames["behavior_frame_id"].values
    x_pos_mm_interp = stage_pos_df_at_muscle_frames["x_pos_mm_interp"].values
    y_pos_mm_interp = stage_pos_df_at_muscle_frames["y_pos_mm_interp"].values
    acquired_time_us = []
    received_time_us = []
    for muscle_path in muscle_image_paths:
        metadata_path = str(muscle_path).replace(".tif", ".csv")
        frame_ds = pd.read_csv(metadata_path).iloc[0]
        acquired_time_us.append(frame_ds["acquired_time_us"])
        received_time_us.append(frame_ds["received_time_us"])

    return pd.DataFrame(
        data={
            "muscle_frame_id": muscle_frame_ids.astype(np.uint32),
            "corresponding_behavior_frame_id": behavior_frame_ids.astype(np.uint32),
            "x_pos_mm_interp": x_pos_mm_interp.astype(np.float32),
            "y_pos_mm_interp": y_pos_mm_interp.astype(np.float32),
            "acquired_time_us": np.array(acquired_time_us, dtype=np.uint64),
            "received_time_us": np.array(received_time_us, dtype=np.uint64),
        }
    )


# if __name__ == "__main__":
#     logging.basicConfig(
#         level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
#     )

#     # fmt: off
#     recording_dir = Path("/home/sibwang/Data/spotlight/20250613-fly1b-002/")
#     map_muscle_frames_to_behavior(
#         muscle_calibration_path=recording_dir / "metadata/calibration_parameters_muscle.yaml",
#         behavior_calibration_path=recording_dir / "metadata/calibration_parameters_behavior.yaml",
#         dual_recording_timing_path=recording_dir / "metadata/dual_recording_timing.yaml",
#         processed_behavior_frame_metadata_path=recording_dir / "processed/behavior_frames_metadata.csv",
#         behavior_alignment_metadata_path=recording_dir / "processed/behavior_alignment_transforms.h5",
#         raw_muscle_images_dir=recording_dir / "muscle_images/",
#         transformed_muscle_images_output_dir=recording_dir / "processed/muscle_images/",
#         muscle_metadata_output_path=recording_dir / "processed/muscle_frames_metadata.csv",
#     )
#     # fmt: on
