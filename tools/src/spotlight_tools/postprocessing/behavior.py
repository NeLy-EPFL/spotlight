import cv2
import h5py
import numpy as np
import logging
import yaml
from pathlib import Path
from tqdm import tqdm
from tempfile import TemporaryDirectory
from sleap_io import Video
from sleap_nn.predict import run_inference
from joblib import Parallel, delayed

import spotlight_tools
from spotlight_tools.common.video import write_video

logging.basicConfig(level=logging.INFO)


def expand_and_align_frames_single_batch(
    pseudo3ch_frame_paths: list[Path],
    sleap_model_dir: Path,
    output_path_stem: Path,
    use_shm: bool,
    keypoints_code2name: dict[str, str],
    crop_dim: int = 900,
    play_fps: int = 30,
    behavior_video_crf: int = 5,
    behavior_video_preset: str = "slow",
):
    # with TemporaryDirectory(dir="/dev/shm" if use_shm else None) as tmpdir:
    if True:
        tmpdir = "/dev/shm/sleap_temp"
        logging.info(
            f"Processing {len(pseudo3ch_frame_paths)} pseudo-RGB frames under "
            f"temporary directory {tmpdir}"
        )

        # Convert pseudo 3-channel images to sequences of 3 monochrome images
        expanded_frames_dir = Path(tmpdir) / "single_channel_frames"
        expanded_frames_dir.mkdir(parents=True, exist_ok=True)
        expanded_frame_paths, real1ch_frames = _expand_pseudo3ch(
            expanded_frames_dir, return_frames=True
        )

        # Expand pseudo 3-channel frames into separate 1-channel frames
        keypoints_xy_pre_align = _estimate_2dpose(
            expanded_frame_paths,
            sleap_model_dir,
            keypoints_code2name,
            output_path=Path(tmpdir) / "sleap_predictions.slp",  # will be deleted
        )

        # Transform frame by frame based on keypoints
        transformed_frames, transformed_keypoints, transform_matrices = (
            _transform_all_behavior_frames(
                keypoints_xy_pre_align,
                expanded_frame_paths,
                real1ch_frames,
                keypoints_code2name,
                crop_dim,
            )
        )

        # Save transformed frames as video and transformation metadata
        output_path_stem.parent.mkdir(parents=True, exist_ok=True)
        write_video(
            output_path=output_path_stem.with_suffix(".mkv"),
            frames=transformed_frames,
            fps=play_fps,
            crf=behavior_video_crf,
            preset=behavior_video_preset,
        )
        _save_transformation_metadata(
            output_path=output_path_stem.with_suffix(".h5"),
            keypoints_xy_pre_align=keypoints_xy_pre_align,
            transformed_keypoints=transformed_keypoints,
            transform_matrices=transform_matrices,
            keypoints_code2name=keypoints_code2name,
        )


def _expand_pseudo3ch(
    real1ch_frames_dir, return_frames: bool
) -> list[Path] | tuple[list[Path], list[np.ndarray]]:
    """Spotlight saves 3 adjacent behavior images as a single pseudo-RGB
    JPEG image (an IO optimization trick). This function expands it into
    separate monochrome images.

    Returns:
        list[Path]: List of paths to the expanded single-channel images.
        (Only if return_frames is True) list[np.ndarray]: List of the
            expanded single-channel images.
    """
    real1ch_frame_paths = []
    real1ch_frames = []
    for i, pseudo3ch_frame_path in enumerate(pseudo3ch_frame_paths):
        pseudo3ch_image = cv2.imread(str(pseudo3ch_frame_path))
        blue, green, red = cv2.split(pseudo3ch_image)
        for j, channel in enumerate([blue, green, red]):
            frame_id = i * 3 + j
            real1ch_frame_path = real1ch_frames_dir / f"frame_{frame_id:06d}.jpg"
            cv2.imwrite(str(real1ch_frame_path), channel)
            real1ch_frame_paths.append(real1ch_frame_path)
            if return_frames:
                real1ch_frames.append(channel)
    if return_frames:
        return real1ch_frame_paths, real1ch_frames
    else:
        return real1ch_frame_paths


def _estimate_2dpose(
    real1ch_frame_paths: list[Path],
    sleap_model_dir: Path,
    keypoints_code2name: dict[str, str],
    output_path: Path,
) -> np.ndarray:
    """Run SLEAP 2D pose estimation on single-channel images."""
    # Run SLEAP inference
    video = Video.from_filename([str(path) for path in real1ch_frame_paths])
    predicted_labels = run_inference(
        input_video=video, model_paths=[sleap_model_dir], output_path=output_path
    )

    # Format results
    keypoints_code2idx = {
        code: idx for idx, code in enumerate(keypoints_code2name.keys())
    }
    keypoints_xy = np.full(
        (len(real1ch_frame_paths), len(keypoints_code2name), 2), np.nan
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


def _fill_keypoints_gaps(
    keypoints_xy: np.ndarray,
) -> np.ndarray:
    """If any keypoint in a frame is NaN, fill it with the last valid
    keypoints. If the 0th frame is NaN, fill it with the first valid keypoints.
    """
    num_frames, _, _ = keypoints_xy.shape
    keypoints_xy_filled = keypoints_xy.copy()

    # Find the first valid frame
    first_valid_frame = None
    for i in range(num_frames):
        if not np.isnan(keypoints_xy_filled[i, ...]).any():
            first_valid_frame = i
            break

    if first_valid_frame is None:
        logging.error("All frames have NaN keypoints. Cannot fill gaps.")
        return keypoints_xy_filled

    # Fill leading NaNs with the first valid frame
    for i in range(first_valid_frame):
        keypoints_xy_filled[i, ...] = keypoints_xy_filled[first_valid_frame, ...]

    # Fill intermediate NaNs with the last valid frame
    for i in range(first_valid_frame + 1, num_frames):
        if np.isnan(keypoints_xy_filled[i, ...]).any():
            keypoints_xy_filled[i, ...] = keypoints_xy_filled[i - 1, ...]

    return keypoints_xy_filled


def _transform_single_frame(
    input_frame: np.ndarray,
    keypoints: np.ndarray,
    crop_dim: int,
    thorax_idx: int,
    neck_idx: int,
    abdomen_idx: int,
) -> np.ndarray:
    """Transform a single frame based on keypoints."""
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


def _transform_all_behavior_frames(
    keypoints_xy_pre_align: np.ndarray,
    expanded_frame_paths: list[Path],
    real1ch_frames: list[np.ndarray],
    keypoints_code2name: dict[str, str],
    crop_dim: int,
) -> tuple[list[np.ndarray], np.ndarray, np.ndarray]:
    """Transform all behavior frames based on keypoints."""
    keypoints_xy_pre_align_filled = _fill_keypoints_gaps(keypoints_xy_pre_align)
    num_frames = len(expanded_frame_paths)
    thorax_idx = list(keypoints_code2name.values()).index("thorax")
    neck_idx = list(keypoints_code2name.values()).index("neck")
    abdomen_idx = list(keypoints_code2name.values()).index("abdomen tip")

    def _process_frame(i):
        input_frame = real1ch_frames[i]
        keypoints = keypoints_xy_pre_align_filled[i, :, :]
        return _transform_single_frame(
            input_frame, keypoints, crop_dim, thorax_idx, neck_idx, abdomen_idx
        )

    results = Parallel(n_jobs=-1, backend="loky")(
        delayed(_process_frame)(i) for i in range(num_frames)
    )
    transformed_frames, transformed_keypoints, transform_matrices = zip(*results)
    transformed_frames = list(transformed_frames)
    transformed_keypoints = np.array(transformed_keypoints)
    transform_matrices = np.array(transform_matrices)

    return transformed_frames, transformed_keypoints, transform_matrices


def _save_transformation_metadata(
    output_path: Path,
    keypoints_xy_pre_align: np.ndarray,
    transformed_keypoints: np.ndarray,
    transform_matrices: np.ndarray,
    keypoints_code2name: dict[str, str],
):
    with h5py.File(output_path, "w") as hf:
        ds = hf.create_dataset(
            "keypoints_xy_before_alignment",
            data=keypoints_xy_pre_align,
            compression="gzip",
            dtype="float32",
        )
        ds.attrs["keypoint_names"] = list(keypoints_code2name.values())
        ds = hf.create_dataset(
            "keypoints_xy_after_alignment",
            data=transformed_keypoints,
            compression="gzip",
        )
        ds.attrs["keypoint_names"] = list(keypoints_code2name.values())
        hf.create_dataset(
            "transform_matrices", data=transform_matrices, compression="gzip"
        )


def _load_config() -> dict:
    spotlight_package_dir = Path(spotlight_tools.__path__[0]).expanduser()
    config_path = spotlight_package_dir.parent.parent / "config/config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(
            f"Configuration file {config_path} does not exist. Make sure the "
            "spotlight-tools package is installed correctly."
        )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


if __name__ == "__main__":
    raw_beh_dir = Path(
        "/home/sibwang/Data/spotlight/20250613-fly1b-002/behavior_images/"
    )
    sleap_model_dir = Path(
        "/home/sibwang/Data/sleap/models/spotlight_3pt_20251023/models/251024_023711.single_instance.n=900/"
    )
    pseudo3ch_frame_paths = sorted(raw_beh_dir.glob("behavior_frame_*.jpg"))[:90]
    config = _load_config()
    expand_and_align_frames_single_batch(
        pseudo3ch_frame_paths,
        sleap_model_dir,
        output_path_stem=Path("test"),
        use_shm=True,
        keypoints_code2name=config["pose2d"]["keypoint_names"],
        crop_dim=900,
        play_fps=30,
    )
