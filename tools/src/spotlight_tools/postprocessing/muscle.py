"""
Muscle frame processing: warp raw PCO-camera TIFF frames into the behavior
camera's coordinate system and apply the same fly-alignment transforms used
for the behavior channel.

Spatial mapping uses the legacy spotlight_tools.calibration module
(SpotlightPositionMapper / BehaviorMuscleCrossMapper), which reads
calibration_parameters_behavior.yaml and calibration_parameters_muscle.yaml
saved alongside each recording by the recorder GUI.  An optional homography
path (homography_parameters.yaml) can be passed instead if a homography
calibration has been performed.
"""

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
    HomographyMapper,
)
from spotlight_tools.common.video import get_video_info
from spotlight_tools.postprocessing.io import check_output_path_against_alignment_flag


_imwrite_compression_params = [cv2.IMWRITE_TIFF_COMPRESSION, 5]
# TIFF compression methods:
#   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
#   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
#   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
#   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
#   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html
# cv::IMWRITE_TIFF_COMPRESSION_LZW is used by the recording GUI


def warp_all_muscle_frames_to_behavior(
    *,
    raw_muscle_images_dir: Path,
    transformed_muscle_images_output_dir: Path,
    muscle_calib_path: Path,
    behavior_calib_path: Path,
    experiment_parameters_path: Path,
    processed_behavior_frame_metadata_path: Path,
    muscle_metadata_output_path: Path,
    align_fly: bool = True,
    behavior_alignment_metadata_path: Path | None = None,
    processed_behavior_video_path: Path | None = None,
    missing_muscle_frames_tolerance: int = 3,
    num_workers: int = -1,
    use_homography: bool = False,
    homography_path: Path | None = None,
):
    """This function performs muscle-to-behavior frame mapping and transformation:

    1. Determines timing synchronization between muscle and behavior recordings.
    2. Spatially maps muscle images to behavior coordinate system using either:
       a. Homography transformation (if use_homography=True)
       b. Spotlight calibration parameters with affine approximation (if use_homography=False)
    3. Applies the same alignment transformations used for behavior frames (if any has
       been applied).
    4. Saves transformed muscle images in TIFF format and metadata.

    Args:
        raw_muscle_images_dir (Path): Directory containing raw muscle image files.
        transformed_muscle_images_output_dir (Path): Directory to save output frames.
        muscle_calib_path (Path): Path to muscle camera calibration parameters.
        behavior_calib_path (Path): Path to behavior camera calibration parameters.
        experiment_parameters_path (Path): Path to the experiment parameters metadata.
        processed_behavior_frame_metadata_path (Path): Path to behavior frames metadata
            (CSV).
        muscle_metadata_output_path (Path): Output path for muscle frames metadata (CSV).
        align_fly (bool): Whether behavior frames have been transformed to align the
            fly and crop the image. If True, `behavior_alignment_metadata_path` must be
            provided. If False, `processed_behavior_video_path` must be provided.
            Default is True.
        behavior_alignment_metadata_path (Path | None): Path to behavior alignment
            transforms (HDF5). Required if `align_fly` is True, ignored otherwise.
        processed_behavior_video_path (Path | None): Path to processed behavior video
            (MP4). Used only to get output dimensions if `align_fly` is False. Ignored
            if `align_fly` is True (output dimensions are taken from alignment metadata
            instead).
        missing_muscle_frames_tolerance (int): See
            `scripts.postprocess_recording.postprocess_recording_data`.
        num_workers (int): Number of parallel workers (-1 for all available cores).
        use_homography (bool): If True, use homography transformation for alignment.
            If False, use stage-dependent affine transformation from Spotlight calibration.
            Default is False.
        homography_path (Path | None): Path to homography calibration YAML file.
            Required if use_homography is True. If None and use_homography is True,
            will look for homography_result.yaml in the standard calibration location.

    Returns:
        None: Outputs are saved to the specified directories and files.
    """
    logger = logging.getLogger(__name__)

    # Check if output path suggests alignment status consistent with `align_fly`
    check_output_path_against_alignment_flag(
        transformed_muscle_images_output_dir, align_fly
    )

    # Get behavior-muscle sync ratio
    behavior_muscle_sync_ratio = get_behavior_muscle_sync_ratio(
        experiment_parameters_path=experiment_parameters_path
    )

    # Load the stage positions at muscle recording frames
    stage_pos_df_at_muscle_frames = _get_stage_pos_df_at_muscle_frames(
        processed_behavior_frame_metadata_path, behavior_muscle_sync_ratio
    )

    # Create mapping object(s)
    if use_homography:
        # Use homography-based mapping
        if homography_path is None:
            # Try to find homography file in standard location
            # Assume it's in the profile calibration directory
            homography_path = (
                behavior_calib_path.parent / "metadata/homography_parameters.yaml"
            )
            if not homography_path.exists():
                raise FileNotFoundError(
                    f"Homography file not found at {homography_path}. "
                    "Please provide homography_path or run homography calibration first."
                )
        logger.info(f"Using homography transformation from {homography_path}")
        homography_mapper = HomographyMapper(homography_path)
        cross_mapper = None
    else:
        # Use stage-dependent affine mapping
        logger.info(
            "Using stage-dependent affine transformation from Spotlight calibration"
        )
        behavior_mapper = SpotlightPositionMapper(behavior_calib_path)
        muscle_mapper = SpotlightPositionMapper(muscle_calib_path)
        cross_mapper = BehaviorMuscleCrossMapper(behavior_mapper, muscle_mapper)
        homography_mapper = None

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
    if align_fly:
        alignment_transforms, output_dim = _load_alignment_transform_metadata(
            behavior_alignment_metadata_path, behavior_muscle_sync_ratio
        )
    else:
        ident_transform = np.eye(2, 3)
        alignment_transforms = np.repeat(
            ident_transform[None, :, :], len(muscle_image_paths), axis=0
        )
        width, height, _ = get_video_info(processed_behavior_video_path)
        output_dim = (width, height)

    # Prepare input kwargs for parallel processing
    input_kwargs = []
    for i, input_path in enumerate(muscle_image_paths):
        stage_pos = stage_pos_df_at_muscle_frames.iloc[i][
            ["x_pos_mm_interp", "y_pos_mm_interp"]
        ].values.astype(np.float32)

        if use_homography:
            # Use homography transformation (stage-independent)
            muscle2behavior_transform_matrix = homography_mapper.H_muscle2beh
            use_perspective = True
        else:
            # Use stage-dependent affine transformation
            muscle2behavior_transform_matrix = (
                cross_mapper.get_affine_matrix_muscle2behavior(stage_pos)
            )
            use_perspective = False

        behavior_alignment_transform_matrix = alignment_transforms[i]
        output_path = transformed_muscle_images_output_dir / input_path.name
        kwargs = {
            "muscle2behavior_trans_mat": muscle2behavior_transform_matrix,
            "behavior_alignment_trans_mat": behavior_alignment_transform_matrix,
            "input_path": input_path,
            "output_dim": output_dim,
            "output_path": output_path,
            "return_output": False,  # reduce IO stress
            "use_perspective": use_perspective,
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
        delayed(warp_single_muscle_frame_to_behavior)(**kwargs)
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


def get_behavior_muscle_sync_ratio(
    *,
    experiment_parameters_path: Path | str | None,
    recording_dir: Path | str | None = None,
) -> int:
    """Extract the behavior-to-muscle frame synchronization ratio.

    Reads ``muscle_sync_ratio`` from ``experiment_parameters.yaml``. For backward
    compatibility with recordings made before the timing metadata was merged into
    ``experiment_parameters.yaml``, it also accepts the legacy ``sync_ratio`` key
    (written by the now-removed ``dual_recording_timing.yaml``), emitting a
    deprecation warning. ``experiment_parameters_path`` may point at either file.
    """
    if experiment_parameters_path is None and recording_dir is None:
        raise ValueError(
            "Either experiment_parameters_path or recording_dir must be provided."
        )
    if experiment_parameters_path is not None and recording_dir is not None:
        raise ValueError(
            "Only one of experiment_parameters_path or recording_dir should be provided."
        )

    logger = logging.getLogger(__name__)

    def _read_sync_ratio(path: Path) -> int | None:
        """Return the sync ratio stored in `path`, or None if it is not there.

        Accepts both the current ``experiment_parameters.yaml`` (``muscle_sync_ratio``)
        and the legacy ``dual_recording_timing.yaml`` (``sync_ratio``), so the caller
        may pass either file.
        """
        if not path.exists():
            return None
        with open(path, "r") as f:
            metadata = yaml.safe_load(f) or {}
        if "muscle_sync_ratio" in metadata:
            return metadata["muscle_sync_ratio"]
        if "sync_ratio" in metadata:  # legacy dual_recording_timing.yaml
            logger.warning(
                "DEPRECATION: reading the behavior-muscle sync ratio from the legacy "
                "'sync_ratio' key in '%s'. The recorder now writes 'muscle_sync_ratio' "
                "to 'experiment_parameters.yaml'; legacy support will be removed in a "
                "future release.",
                path,
            )
            return metadata["sync_ratio"]
        return None

    # Resolve the metadata file(s) to try, in priority order. A caller may point
    # experiment_parameters_path at either the current experiment_parameters.yaml or
    # the legacy dual_recording_timing.yaml -- _read_sync_ratio handles both keys. When
    # only recording_dir is given, try the current file, then the legacy sibling.
    if experiment_parameters_path is not None:
        explicit_path = Path(experiment_parameters_path)
        candidate_paths = [
            explicit_path,
            explicit_path.parent / "dual_recording_timing.yaml",
        ]
    else:
        metadata_dir = Path(recording_dir) / "metadata"
        candidate_paths = [
            metadata_dir / "experiment_parameters.yaml",
            metadata_dir / "dual_recording_timing.yaml",
        ]
    candidate_paths = list(dict.fromkeys(candidate_paths))  # dedupe, preserve order

    for path in candidate_paths:
        sync_ratio = _read_sync_ratio(path)
        if sync_ratio is not None:
            return sync_ratio

    raise FileNotFoundError(
        "Could not determine the behavior-muscle sync ratio from any of: "
        + ", ".join(f"'{p}'" for p in candidate_paths)
    )


def _get_stage_pos_df_at_muscle_frames(
    interpolated_stage_pos_path: Path, behavior_muscle_sync_ratio: int
) -> pd.DataFrame:
    stage_pos_df_at_behavior_frames = pd.read_csv(interpolated_stage_pos_path)
    stage_pos_df_at_muscle_frames = stage_pos_df_at_behavior_frames[
        behavior_muscle_sync_ratio::behavior_muscle_sync_ratio
    ]
    assert (
        match_muscle_frameid_to_behavior_frameid(
            0, sync_ratio=behavior_muscle_sync_ratio
        )
        == stage_pos_df_at_muscle_frames.iloc[0]["behavior_frame_id"]
    ), "Muscle-to-behavior frame ID mapping mismatch."
    return stage_pos_df_at_muscle_frames


def _filter_muscle_frames_by_availability(
    raw_muscle_images_dir: Path,
    stage_pos_df_at_muscle_frames: pd.DataFrame,
    missing_muscle_frames_tolerance: int = 3,
) -> list[Path]:
    # Index all available frames
    _muscle_paths_by_frameid = {}
    for path in raw_muscle_images_dir.glob("*.tif"):
        try:
            frameid = int(path.stem.split("_")[-1])
        except ValueError:
            logging.warning(
                f"Problem scanning muscle images: Could not parse frame index from "
                f"file name {path.name}. Skipping this file."
            )
            continue
        _muscle_paths_by_frameid[frameid] = path

    # Check if each expected frame is among the frames found
    muscle_image_paths = []
    num_expected_frames = stage_pos_df_at_muscle_frames.shape[0]
    for frameid in range(num_expected_frames):
        if frameid not in _muscle_paths_by_frameid:
            if frameid >= num_expected_frames - missing_muscle_frames_tolerance:
                # If we are almost at the end of the recording, it's ok. This could
                # simply be due to expected synchronization/timing imperfections.
                break
            logging.error(
                f"Problem scanning muscle images: Frame {frameid} not found in "
                f"{raw_muscle_images_dir} (a total of {num_expected_frames} is "
                f"expected). Dataset is incomplete."
            )
            raise RuntimeError("Dataset is incomplete.")
        else:
            muscle_image_paths.append(_muscle_paths_by_frameid[frameid])

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


def warp_single_muscle_frame_to_behavior(
    muscle2behavior_trans_mat: np.ndarray,
    behavior_alignment_trans_mat: np.ndarray,
    input_path: Path,
    output_dim: tuple[int, int],
    output_path: Path,
    return_output: bool = True,
    use_perspective: bool = False,
):
    """Apply composed transformation to align a single muscle frame with the
    corresponding behavior frame.

    Args:
        muscle2behavior_trans_mat (np.ndarray): Transformation matrix mapping muscle
            to behavior coordinates. Can be either:
            - 2x3 affine matrix (from Spotlight calibration)
            - 3x3 homography matrix (from ChArUco calibration)
        behavior_alignment_trans_mat (np.ndarray): 2x3 transformation matrix for
            behavior frame alignment (whatever that's been applied to the behavior
            frame; this can be read out from behavior alignment transform metadata).
        input_path (Path): Path to the input muscle image file.
        output_dim (tuple[int, int]): Output image dimensions (width, height).
        output_path (Path): Path where the transformed image will be saved.
        return_output (bool): Whether to return the transformed image array. Not
            returning anything may help reduce IO load if called in parallel (automatic
            garbage collection might not happen until after the map operation).
        use_perspective (bool): If True, treat muscle2behavior_trans_mat as a 3x3
            homography matrix and compose with affine alignment. If False, treat it
            as a 2x3 affine matrix. Default is False.

    Returns:
        np.ndarray or None (depending on return_output): Transformed muscle frame.
    """
    # Load muscle image
    in_image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)

    if use_perspective:
        # Homography case: muscle2behavior_trans_mat is 3x3
        # First apply homography, then apply affine alignment
        # We need to compose: alignment @ homography

        # Ensure homography is 3x3
        if muscle2behavior_trans_mat.shape == (2, 3):
            muscle2behavior_trans_mat = np.vstack(
                [muscle2behavior_trans_mat, [0, 0, 1]]
            )

        # Convert affine alignment to 3x3 homogeneous
        behavior_alignment_trans_mat_3x3 = np.vstack(
            [behavior_alignment_trans_mat, [0, 0, 1]]
        )

        # Compose: first homography, then alignment
        composed_trans_mat = (
            behavior_alignment_trans_mat_3x3 @ muscle2behavior_trans_mat
        )

        # Apply perspective warp
        out_image = cv2.warpPerspective(in_image, composed_trans_mat, output_dim)
    else:
        # Affine case: both are 2x3, compose them
        # Convert 2x3 transform matrices to 3x3 homogeneous matrices for easier composition
        muscle2behavior_trans_mat_3x3 = np.vstack(
            [muscle2behavior_trans_mat, [0, 0, 1]]
        )
        behavior_alignment_trans_mat_3x3 = np.vstack(
            [behavior_alignment_trans_mat, [0, 0, 1]]
        )

        # Compose transforms: first muscle->behavior, then apply the same alignment
        # transform that was applied to the behavior frame.
        composed_trans_mat = (
            behavior_alignment_trans_mat_3x3 @ muscle2behavior_trans_mat_3x3
        )
        assert np.allclose(composed_trans_mat[2, :], [0, 0, 1])

        # Convert back to 2x3 for cv2.warpAffine
        composed_trans_mat = composed_trans_mat[:2, :]

        # Apply affine warp
        out_image = cv2.warpAffine(in_image, composed_trans_mat, output_dim)

    # Save output image
    cv2.imwrite(str(output_path), out_image, _imwrite_compression_params)

    if return_output:
        return out_image
    else:
        # don't return anything - might help reduce IO traffic if called in parallel
        return None


def _make_muscle_metadata_dataframe(muscle_image_paths, stage_pos_df_at_muscle_frames):
    muscle_frameids = np.arange(len(muscle_image_paths))
    behavior_frameids = stage_pos_df_at_muscle_frames["behavior_frame_id"].values
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
            "muscle_frame_id": muscle_frameids.astype(np.uint32),
            "corresponding_behavior_frame_id": behavior_frameids.astype(np.uint32),
            "x_pos_mm_interp": x_pos_mm_interp.astype(np.float32),
            "y_pos_mm_interp": y_pos_mm_interp.astype(np.float32),
            "acquired_time_us": np.array(acquired_time_us, dtype=np.uint64),
            "received_time_us": np.array(received_time_us, dtype=np.uint64),
        }
    )


def match_muscle_frameid_to_behavior_frameid(
    muscle_frameid: int | list[int],
    *,
    sync_ratio: int | None = None,
    experiment_parameters_path: Path | None = None,
    recording_dir: Path | None = None,
):
    """Map muscle frame ID or IDs to corresponding behavior frame ID(s) using the
    provided synchronization ratio.

    Note that muscle recording lags behind behavior recording by one cycle. For example,
    if the sync ratio is 10, then the 0th muscle frame is recorded at the same time as
    the 10th behavior frame (this is handled internally by this function).
    """
    if sync_ratio is None:
        sync_ratio = get_behavior_muscle_sync_ratio(
            experiment_parameters_path=experiment_parameters_path,
            recording_dir=recording_dir,
        )

    is_singleton_int = isinstance(muscle_frameid, (int, np.integer))
    if is_singleton_int:
        muscle_frameid = [muscle_frameid]

    behavior_frameid = [(mfid + 1) * sync_ratio for mfid in muscle_frameid]

    if is_singleton_int:
        behavior_frameid = behavior_frameid[0]
    return behavior_frameid


def match_behavior_frameid_to_muscle_frameid(
    behavior_frameid: int | list[int],
    method: str,
    *,
    sync_ratio: int | None = None,
    experiment_parameters_path: Path | None = None,
    recording_dir: Path | None = None,
):
    """Map behavior frame ID or IDs to corresponding muscle frame ID(s) using the
    provided synchronization ratio.

    Because there are more behavior frames than muscle frames, multiple behavior frames
    will correspond to the same muscle frame. The selection of which muscle frame to
    return is controlled by the `method` argument:
    - "floor": use the last available muscle frame.
    - "nearest": use the temporally closest muscle frame (might be in the future).

    Note that muscle recording lags behind behavior recording by one cycle. For example,
    if the sync ratio is 10, then the 0th muscle frame is recorded at the same time as
    the 10th behavior frame (this is handled internally by this function).
    """
    if sync_ratio is None:
        sync_ratio = get_behavior_muscle_sync_ratio(
            experiment_parameters_path=experiment_parameters_path,
            recording_dir=recording_dir,
        )
    if method.lower() not in ["floor", "nearest"]:
        raise ValueError(
            f"Invalid method '{method}'. Supported methods are 'floor' and 'nearest'."
        )

    is_singleton_int = isinstance(behavior_frameid, (int, np.integer))
    if is_singleton_int:
        behavior_frameid = [behavior_frameid]

    muscle_frameid = []
    for bfid in behavior_frameid:
        if method == "floor":
            mfid = int((bfid - sync_ratio) / sync_ratio)
        elif method == "nearest":
            mfid = round((bfid - sync_ratio) / sync_ratio)
        muscle_frameid.append(mfid)

    if is_singleton_int:
        muscle_frameid = muscle_frameid[0]
    return muscle_frameid


# if __name__ == "__main__":
#     logging.basicConfig(
#         level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
#     )

#     # fmt: off
#     recording_dir = Path("~/data/spotlight/20250613-fly1b-002/").expanduser()
#     map_muscle_frames_to_behavior(
#         muscle_calibration_path=recording_dir / "metadata/calibration_parameters_muscle.yaml",
#         behavior_calibration_path=recording_dir / "metadata/calibration_parameters_behavior.yaml",
#         experiment_parameters_path=recording_dir / "metadata/experiment_parameters.yaml",
#         processed_behavior_frame_metadata_path=recording_dir / "processed/behavior_frames_metadata.csv",
#         behavior_alignment_metadata_path=recording_dir / "processed/behavior_alignment_transforms.h5",
#         raw_muscle_images_dir=recording_dir / "muscle_images/",
#         transformed_muscle_images_output_dir=recording_dir / "processed/muscle_images/",
#         muscle_metadata_output_path=recording_dir / "processed/muscle_frames_metadata.csv",
#     )
#     # fmt: on
