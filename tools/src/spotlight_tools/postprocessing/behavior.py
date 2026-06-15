"""
Behavior frame processing: decode pseudo-BGR JPEGs, run SLEAP 2-D pose
estimation, align and crop each frame so the fly faces upward.

The recorder bundles every three consecutive behavior frames into a single
JPEG file (a performance hack at save time); expand_single_pseudo_bgr_image
unpacks each such file before any further processing.
"""

import cv2
import h5py
import numpy as np
import logging
from pathlib import Path
from tempfile import TemporaryDirectory
from sleap_io import Video
from sleap_nn.predict import run_inference
from joblib import Parallel, delayed

from spotlight_tools.common.video import get_video_writer
from spotlight_tools.postprocessing.io import check_output_path_against_alignment_flag


def decode_and_align_all_behavior_frames(
    *,
    raw_behavior_frame_paths: list[Path],
    sleap_model_dir: Path,
    output_video_path: Path,
    output_metadata_path: Path,
    keypoints_code2name: dict[str, str],
    align_fly: bool = True,
    use_shm: bool = False,
    sleap_batch_size: int = 128,
    crop_dim: int = 900,
    play_fps: int = 33,
    behavior_video_crf: int = 12,
    behavior_video_preset: str = "slow",
    num_workers: int = -1,
) -> None:
    """This function processes behavior frames through the following pipeline:

    1. Expands pseudo-BGR JPEG images into separate monochrome frames (during recording,
       every three consecutive frames are saved as a single 3-channel JPEG for
       performance considerations)
    2. (If `align_fly` is True) Runs a 3-keypoint pose estimation with SLEAP to detect
       fly position and heading
    3. (If `align_fly` is True) Transforms frames to align and center the fly (facing
       upward)
    4. Outputs an aligned behavior video and transformation metadata

    Args:
        raw_behavior_frame_paths (list[Path]): Paths to input pseudo-BGR JPEG files.
        sleap_model_dir (Path): Directory containing trained SLEAP model files.
        output_video_path (Path): Path for the output aligned behavior video.
        output_metadata_path (Path): Path for the transformation metadata (HDF5).
        keypoints_code2name (dict[str, str]): Mapping from keypoint codes (used by
            SLEAP) to meaningful names.
        align_fly (bool): If False, skip pose estimation and frame alignment; simply
            decode pseudo-BGR images into separate monochrome frames and save as video.
            Default is True.
        use_shm (bool): Whether to use shared memory (/dev/shm) for temporary files.
            Doing so will avoid duplicated disk read and write, but it is extremely
            sketchy - if the process runs out of shared memory, the entire OS will
            likely crash. Default is False.
        sleap_batch_size (int): Batch size for SLEAP inference.
        crop_dim (int): Output frame dimensions (actual size is crop_dim x crop_dim px).
        play_fps (int): Frame rate for the output video. This is for visualization only
            and has no impact on the actual data (see
            `scripts.postprocess_recording.postprocess_recording_data`).
        behavior_video_crf (int): Constant Rate Factor for video encoding quality. Lower
            is better. 12-17 is visually lossless for most purposes. <10 is overkill.
        behavior_video_preset (str): ffmpeg preset for encoding speed vs compression.
            Slower setting = better compression. "slow" or "slower" is recommended.
        num_workers (int): Number of parallel workers (-1 for all available cores).
    """
    # Set logging verbosity
    logger = logging.getLogger(__name__)

    # Check if output path suggests alignment status consistent with `align_fly`
    check_output_path_against_alignment_flag(output_video_path, align_fly)

    with TemporaryDirectory(dir="/dev/shm" if use_shm else None) as tmpdir:
        logger.info(
            f"Processing {len(raw_behavior_frame_paths)} pseudo-BGR frames under "
            f"temporary directory {tmpdir}"
        )

        # Convert pseudo 3-channel images to sequences of 3 monochrome images
        logger.info("Expanding pseudo-BGR images to single-channel frames")
        expanded_frames_dir = Path(tmpdir) / "single_channel_frames"
        expanded_frames_dir.mkdir(parents=True, exist_ok=True)
        single_channel_frame_paths = _expand_all_pseudo_bgr_images(
            raw_behavior_frame_paths, expanded_frames_dir, num_workers=num_workers
        )

        if align_fly:
            # Run SLEAP 2D pose estimation on single-channel frames
            logger.info("Running SLEAP 2D pose estimation on expanded frames")
            keypoints_xy_pre_alignment = estimate_2dpose_sequence(
                single_channel_frame_paths,
                sleap_model_dir,
                keypoints_code2name,
                output_path=Path(tmpdir) / "sleap_predictions.slp",  # will be deleted
                batch_size=sleap_batch_size,
            )

            # Transform frame by frame based on keypoints
            logger.info("Transforming behavior frames to align the fly")
            transformed_frame_paths, transformed_keypoints, transform_matrices = (
                _transform_all_frames_to_align(
                    keypoints_xy_pre_alignment,
                    single_channel_frame_paths,
                    keypoints_code2name,
                    crop_dim,
                    output_dir=Path(tmpdir) / "aligned_frames",
                    num_workers=num_workers,
                )
            )

        # Save transformed frames as video
        output_video_path.parent.mkdir(parents=True, exist_ok=True)
        logger.info(f"Saving aligned behavior video to {output_video_path}")
        video_writer = get_video_writer(
            output_path=output_video_path,
            fps=play_fps,
            crf=behavior_video_crf,
            preset=behavior_video_preset,
            logging=logger.level <= logging.INFO,
        )
        if align_fly:
            frame_paths_to_write = transformed_frame_paths
        else:
            frame_paths_to_write = single_channel_frame_paths
        for frame_path in frame_paths_to_write:
            frame = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
            video_writer.write(frame)
        video_writer.close()

        # Save transformation metadata if fly alignment was performed
        if align_fly:
            logger.info(f"Saving transformation metadata to {output_metadata_path}")
            output_metadata_path.parent.mkdir(parents=True, exist_ok=True)
            _save_transformation_metadata(
                output_path=output_metadata_path,
                keypoints_xy_pre_alignment=keypoints_xy_pre_alignment,
                transformed_keypoints=transformed_keypoints,
                transform_matrices=transform_matrices,
                keypoints_code2name=keypoints_code2name,
                output_dim=(crop_dim, crop_dim),
            )


def _expand_all_pseudo_bgr_images(
    pseudo3ch_frame_paths: list[Path],
    single_channel_frames_dir: Path,
    num_workers: int = -1,
) -> list[Path]:
    """Expand all pseudo-BGR images into individual single-channel frames in parallel.

    Args:
        pseudo3ch_frame_paths (list[Path]): Paths to pseudo-BGR input images.
        single_channel_frames_dir (Path): Directory where expanded frames will be saved.
        num_workers (int): Number of parallel workers (-1 for all available cores).

    Returns:
        list[Path]: Paths to all generated single-channel frame files.
    """
    logger = logging.getLogger(__name__)
    verbosity = 1 if logger.level <= logging.INFO else 0

    single_channel_frames_dir.mkdir(parents=True, exist_ok=True)
    parallel_mapper = Parallel(n_jobs=num_workers, backend="loky", verbose=verbosity)
    logger.info(
        f"Expanding pseudo-BGR images using {num_workers} "
        f"(effectively {parallel_mapper._effective_n_jobs()}) workers"
    )
    single_channel_frame_paths_grouped = parallel_mapper(
        delayed(expand_single_pseudo_bgr_image)(
            input_path, single_channel_frames_dir / f"frame_{i:06d}"
        )
        for i, input_path in enumerate(pseudo3ch_frame_paths)
    )
    logger.info("Finished expanding pseudo-BGR images")
    single_channel_frame_paths = []
    for group in single_channel_frame_paths_grouped:
        single_channel_frame_paths.extend(group)
    return single_channel_frame_paths


def expand_single_pseudo_bgr_image(
    pseudo3ch_frame_path: Path, output_path_stem: Path
) -> list[Path]:
    """Spotlight saves 3 adjacent behavior images as a single pseudo-BGR
    JPEG image (an IO optimization trick). This function expands it into
    separate monochrome images.

    Args:
        pseudo3ch_frame_path: Path to the input pseudo-BGR image.
        output_path_stem: Path stem for the output single-channel images.
            The function will append suffixes "_ch0.jpg", "_ch1.jpg", "_ch2.jpg"
            for the three channels.

    Returns:
        list of Paths to the three output single-channel images.
    """
    pseudo3ch_image = cv2.imread(str(pseudo3ch_frame_path))
    blue, green, red = cv2.split(pseudo3ch_image)
    output_paths = []
    for j, channel in enumerate([blue, green, red]):
        single_channel_frame_path = output_path_stem.with_suffix(f".ch{j}.jpg")
        cv2.imwrite(str(single_channel_frame_path), channel)
        output_paths.append(single_channel_frame_path)
    return output_paths


def estimate_2dpose_sequence(
    single_channel_frame_paths: list[Path],
    sleap_model_dir: Path,
    keypoints_code2name: dict[str, str],
    output_path: Path,
    batch_size: int = 128,
) -> np.ndarray:
    """Run SLEAP 2D pose estimation on single-channel behavior images.

    Args:
        single_channel_frame_paths (list[Path]): Paths to single-channel frame images.
        sleap_model_dir (Path): Directory containing trained SLEAP model files.
        keypoints_code2name (dict[str, str]): Mapping from keypoint codes (used by
            SLEAP) to meaningful names.
        output_path (Path): Path where SLEAP predictions will be (temporarily) saved.
        batch_size (int): Batch size for SLEAP inference processing.

    Returns:
        np.ndarray: Keypoint coordinates with shape (num_frames, num_keypoints, 2).
            NaN values indicate missing keypoints.
    """
    logger = logging.getLogger(__name__)

    # Run SLEAP inference
    logger.info("Running SLEAP 2D pose estimation")
    video = Video.from_filename([str(path) for path in single_channel_frame_paths])
    predicted_labels = run_inference(
        input_video=video,
        model_paths=[sleap_model_dir],
        batch_size=batch_size,
        output_path=output_path,
    )
    logger.info("Finished SLEAP 2D pose estimation")

    # Format results
    keypoints_code2idx = {
        code: idx for idx, code in enumerate(keypoints_code2name.keys())
    }
    keypoints_xy = np.full(
        (len(single_channel_frame_paths), len(keypoints_code2name), 2), np.nan
    )
    for i, frame_labels in enumerate(predicted_labels):
        n_instances = len(frame_labels.instances)
        if n_instances == 0:
            continue  # leave as NaN if no instances detected
        elif n_instances == 1:
            points = frame_labels.instances[0].points
            for point in points:
                keypoint_idx = keypoints_code2idx[point["name"]]
                keypoints_xy[i, keypoint_idx, :] = point["xy"]
        else:
            raise RuntimeError(
                f"Multiple instances ({n_instances}) detected in frame {i}. "
                "This is unexpected for single-animal tracking."
            )
    return keypoints_xy  # (num_frames, num_keypoints, 2)


def fill_gaps_in_2dpose_sequence(keypoints_xy: np.ndarray) -> np.ndarray:
    """Fill missing keypoints (NaN values) using forward and backward filling.

    If any keypoint in a frame is NaN, it is filled with the last valid keypoints.
    If the 0th frame has NaN values, they are filled with the first valid keypoints
    found in subsequent frames.

    Args:
        keypoints_xy (np.ndarray): Keypoint coordinates with shape
            (num_frames, num_keypoints, 2).

    Returns:
        np.ndarray: Keypoints with gaps filled, same shape as input.
    """
    logger = logging.getLogger(__name__)

    num_frames, _, _ = keypoints_xy.shape
    keypoints_xy_filled = keypoints_xy.copy()

    # Find the first valid frame
    first_valid_frame = None
    for i in range(num_frames):
        if not np.isnan(keypoints_xy_filled[i, ...]).any():
            first_valid_frame = i
            break

    if first_valid_frame is None:
        logger.error("All frames have NaN keypoints. Cannot fill gaps.")
        return keypoints_xy_filled

    # Fill leading NaNs with the first valid frame
    for i in range(first_valid_frame):
        keypoints_xy_filled[i, ...] = keypoints_xy_filled[first_valid_frame, ...]

    # Fill intermediate NaNs with the last valid frame
    for i in range(first_valid_frame + 1, num_frames):
        if np.isnan(keypoints_xy_filled[i, ...]).any():
            keypoints_xy_filled[i, ...] = keypoints_xy_filled[i - 1, ...]

    return keypoints_xy_filled


def transform_single_frame_to_align(
    input_frame: np.ndarray,
    keypoints: np.ndarray,
    crop_dim: int,
    thorax_idx: int,
    neck_idx: int,
    abdomen_idx: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform a single behavior frame to align and center the fly.

    The transformation rotates the frame so the fly faces upward (head toward
    negative y-axis) and crops around the thorax to center the fly in a square image.

    Args:
        input_frame (np.ndarray): Input frame image as 2D array.
        keypoints (np.ndarray): Keypoint coordinates with shape (num_keypoints, 2).
        crop_dim (int): Output dimensions (actual size is crop_dim x crop_dim pixels).
        thorax_idx (int): Index of the thorax keypoint in SLEAP output.
        neck_idx (int): Index of the neck keypoint in SLEAP output.
        abdomen_idx (int): Index of the abdomen keypoint in SLEAP output.

    Returns:
        np.ndarray: Transformed frame.
        np.ndarray: Transformed keypoints.
        np.ndarray: Transformation matrix.
    """
    # rotation_pivot and heading are both in (x, y), i.e. (col, row)
    rotation_pivot = keypoints[thorax_idx, :]
    heading = keypoints[neck_idx, :] - keypoints[abdomen_idx, :]
    # arctan2 expects inputs in (y, x) order and returns angle from +x in
    # radians (positive = counter-clockwise)
    current_angle = np.rad2deg(np.arctan2(heading[1], heading[0]))  # arctan2(y, x)
    target_angle = -90  # == arctan2(-1, 0) in deg, i.e. facing up (y inverted OpenCV)
    rotation_angle = target_angle - current_angle  # counter-clockwise positive

    # Define affine transformation matrix
    # Step 1: Rotate around thorax so the fly faces up
    # Note: getRotationMatrix2D expects counter-clockwise angles to be positive
    # However, the y axis is inverted in image coordinates, so the "counter-clockwise"
    # angle calculated above needs to be inverted
    transform_matrix = cv2.getRotationMatrix2D(
        rotation_pivot, -rotation_angle, scale=1.0
    )
    # Step 2: Crop around thorax to center the fly in a smaller square image
    translation_x = -rotation_pivot[0] + crop_dim / 2
    translation_y = -rotation_pivot[1] + crop_dim / 2
    transform_matrix[0, 2] += translation_x
    transform_matrix[1, 2] += translation_y

    # Transform image
    output_frame = cv2.warpAffine(
        input_frame,
        transform_matrix,
        (crop_dim, crop_dim),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

    # Transform keypoints
    keypoints_homogeneous = np.hstack([keypoints, np.ones((keypoints.shape[0], 1))])
    transformed_keypoints = (transform_matrix @ keypoints_homogeneous.T).T  # (n_pts, 2)

    return output_frame, transformed_keypoints, transform_matrix


def _transform_all_frames_to_align(
    keypoints_xy_pre_alignment: np.ndarray,
    expanded_frame_paths: list[Path],
    keypoints_code2name: dict[str, str],
    crop_dim: int,
    output_dir: Path,
    num_workers: int = -1,
) -> tuple[list[Path], np.ndarray, np.ndarray]:
    """Transform all behavior frames to align and center the fly in parallel.

    Args:
        keypoints_xy_pre_alignment (np.ndarray): Pre-alignment keypoint coordinates with
            shape (num_frames, num_keypoints, 2).
        expanded_frame_paths (list[Path]): Paths to pre-alignment single-channel frames.
        keypoints_code2name (dict[str, str]): Mapping from keypoint codes (used by
            SLEAP) to meaningful names.
        crop_dim (int): Output frame dimensions (actual size is crop_dim x crop_dim).
        output_dir (Path): Directory where transformed frames will be saved.
        num_workers (int): Number of parallel workers (-1 for all available cores).

    Returns:
        list[Path]: Paths to the transformed frames.
        np.ndarray: Transformed keypoints with shape (num_frames, num_keypoints, 2).
        np.ndarray: Transformation matrices with shape (num_frames, 2, 3).
    """
    logger = logging.getLogger(__name__)
    verbosity = 1 if logger.level <= logging.INFO else 0

    output_dir.mkdir(parents=True, exist_ok=True)

    keypoints_xy_pre_alignment_filled = fill_gaps_in_2dpose_sequence(
        keypoints_xy_pre_alignment
    )
    num_frames = len(expanded_frame_paths)
    thorax_idx = list(keypoints_code2name.values()).index("thorax")
    neck_idx = list(keypoints_code2name.values()).index("neck")
    abdomen_idx = list(keypoints_code2name.values()).index("abdomen tip")

    def _process_frame(i):
        input_frame = cv2.imread(str(expanded_frame_paths[i]), cv2.IMREAD_UNCHANGED)
        keypoints = keypoints_xy_pre_alignment_filled[i, :, :]
        transformed_frame, transformed_keypoints, transform_matrix = (
            transform_single_frame_to_align(
                input_frame, keypoints, crop_dim, thorax_idx, neck_idx, abdomen_idx
            )
        )
        output_path = output_dir / f"aligned_frame_{i:06d}.jpg"
        cv2.imwrite(str(output_path), transformed_frame)
        return output_path, transformed_keypoints, transform_matrix

    parallel_mapper = Parallel(n_jobs=num_workers, backend="loky", verbose=verbosity)
    logger.info(
        f"Transforming behavior images to align the fly using {num_workers} "
        f"(effectively {parallel_mapper._effective_n_jobs()}) workers"
    )
    results = parallel_mapper(delayed(_process_frame)(i) for i in range(num_frames))
    logger.info("Finished transforming behavior images")
    transformed_frame_paths, transformed_keypoints, transform_matrices = zip(*results)
    transformed_frame_paths = list(transformed_frame_paths)
    transformed_keypoints = np.array(transformed_keypoints)
    transform_matrices = np.array(transform_matrices)

    return transformed_frame_paths, transformed_keypoints, transform_matrices


def _save_transformation_metadata(
    output_path: Path,
    keypoints_xy_pre_alignment: np.ndarray,
    transformed_keypoints: np.ndarray,
    transform_matrices: np.ndarray,
    keypoints_code2name: dict[str, str],
    output_dim: tuple[int, int],
):
    """Save detected pose (both pre- and post-alignment), and transformation matrices
    used for alignment to an H5 file."""
    logger = logging.getLogger(__name__)

    logger.info(f"Saving transformation metadata to {output_path}")
    with h5py.File(output_path, "w") as hf:
        ds = hf.create_dataset(
            "keypoints_xy_pre_alignment",
            data=keypoints_xy_pre_alignment,
            compression="gzip",
            dtype="float32",
        )
        ds.attrs["keypoint_names"] = list(keypoints_code2name.values())
        ds = hf.create_dataset(
            "keypoints_xy_post_alignment",
            data=transformed_keypoints,
            compression="gzip",
        )
        ds.attrs["keypoint_names"] = list(keypoints_code2name.values())
        ds = hf.create_dataset(
            "transform_matrices", data=transform_matrices, compression="gzip"
        )
        ds.attrs["output_dim"] = list(output_dim)
    logger.info("Finished saving transformation metadata")


# if __name__ == "__main__":
#     from spotlight_tools.common import load_spotlight_tools_config

#     logging.basicConfig(
#         level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
#     )

#     # fmt: off
#     recording_dir = Path("~/data/spotlight/20250613-fly1b-002/").expanduser()
#     sleap_model_dir = Path(
#         "~/data/sleap/models/spotlight_3pt_20251023/models/251024_023711.single_instance.n=900/"
#     ).expanduser()
#     pseudo3ch_frame_paths = sorted(recording_dir.glob("behavior_images/behavior_frame_*.jpg"))
#     config = load_spotlight_tools_config()
#     decode_and_transform_behavior_frames(
#         pseudo3ch_frame_paths,
#         sleap_model_dir,
#         output_video_path=recording_dir / "processed/aligned_behavior_video.mkv",
#         output_metadata_path=recording_dir / "processed/behavior_alignment_transforms.h5",
#         use_shm=False,
#         keypoints_code2name=config["pose2d"]["keypoint_names"],
#         crop_dim=900,
#         play_fps=33,
#     )
#     # fmt: on
