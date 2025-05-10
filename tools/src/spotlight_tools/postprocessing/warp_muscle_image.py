import logging
import cv2
import numba as nb
import numpy as np
import pandas as pd
import yaml
from collections import defaultdict
from pathlib import Path
from tqdm import tqdm


_imwrite_compression_params = [cv2.IMWRITE_TIFF_COMPRESSION, 5]
# TIFF compression methods:
#   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
#   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
#   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
#   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
#   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html
# cv::IMWRITE_TIFF_COMPRESSION_LZW is used by the recording GUI


class MuscleToBehaviorMapping:
    def __init__(
        self,
        behavior_calibration_path: Path,
        muscle_calibration_path: Path,
        behavior_image_dim: tuple[int, int],
    ):
        """The MuscleToBehaviorMapping class is a wrapper for parameters
        fitted by fit_calibration_model.py. It can also be used to apply
        the transform defined by these parameters to warp the muscle image
        so that it is aligned with the behavior image.

        Args:
            behavior_calibration_params_path (Path):
                Path to the behavior camera calibration parameters. This
                file should be located at "metadata/calibration_parameters_behavior.yaml"
                under the recording directory (the path that you set in the
                GUI). This file is created by the recording GUI at the time
                of recording and is copied from "calibration/model/behavior_camera/"
                under the user profile folder **at the time of recording**
                (i.e., it might change after).
            muscle_calibration_params_path (Path):
                Path to the muscle camera calibration parameters. Same as
                above but replace "behavior" with "muscle" in paths.
            behavior_image_dim (tuple[int, int]):
                Dimensions of the behavior image as (nrows, ncols).
                This is used to create the output image after warping.
        """
        self.behavior_calibration_path = behavior_calibration_path
        self.muscle_calibration_path = muscle_calibration_path
        self.behavior_image_dim = behavior_image_dim
        params = {}
        with open(self.behavior_calibration_path, "r") as f:
            params["behavior_cam"] = yaml.safe_load(f)
        with open(self.muscle_calibration_path, "r") as f:
            params["muscle_cam"] = yaml.safe_load(f)

        input_keys = {
            "stagephys2px": [
                "bias",
                "stage_pos_x",
                "stage_pos_y",
                "physical_pos_x",
                "physical_pos_y",
            ],
            "stagepx2phys": [
                "bias",
                "stage_pos_x",
                "stage_pos_y",
                "pixel_pos_x",
                "pixel_pos_y",
            ],
        }
        transform_shorthands = {
            "stagephys2px": "stage_and_physical_to_pixel",
            "stagepx2phys": "stage_and_pixel_to_physical",
        }

        self.weight_vectors = defaultdict(dict)
        for target_cam in ["behavior_cam", "muscle_cam"]:
            self.weight_vectors[target_cam] = defaultdict(dict)
            for transform in ["stagephys2px", "stagepx2phys"]:
                self.weight_vectors[target_cam][transform] = {}
                for dim in ["x", "y"]:
                    trans = transform_shorthands[transform]
                    target_dim = f"{trans.split('_')[-1]}_pos_{dim}"
                    vec = np.zeros(len(input_keys[transform]))
                    for i, in_key in enumerate(input_keys[transform]):
                        vec[i] = params[target_cam][trans][target_dim][in_key]
                    self.weight_vectors[target_cam][transform][dim] = vec

    def warp_muscle_image(self, muscle_image, x_stage, y_stage):
        return map_muscle_to_behavior_jit(
            muscle_image,
            self.behavior_image_dim,
            x_stage,
            y_stage,
            self.weight_vectors["muscle_cam"]["stagephys2px"]["x"],
            self.weight_vectors["muscle_cam"]["stagephys2px"]["y"],
            self.weight_vectors["behavior_cam"]["stagepx2phys"]["x"],
            self.weight_vectors["behavior_cam"]["stagepx2phys"]["y"],
        )


@nb.jit(nopython=True, fastmath=True)
def get_muscle_px_coords_jit(
    x_stage,
    y_stage,
    row_behavior,
    col_behavior,
    w_mus_stagephys2px_x,
    w_mus_stagephys2px_y,
    w_beh_stagepx2phys_x,
    w_beh_stagepx2phys_y,
):
    input_beh_stagepx2phys = np.array([1, x_stage, y_stage, col_behavior, row_behavior])
    x_phys = np.dot(w_beh_stagepx2phys_x, input_beh_stagepx2phys)
    y_phys = np.dot(w_beh_stagepx2phys_y, input_beh_stagepx2phys)

    input_mus_stagephys2px = np.array([1, x_stage, y_stage, x_phys, y_phys])
    row_muscle = np.dot(w_mus_stagephys2px_y, input_mus_stagephys2px)
    col_muscle = np.dot(w_mus_stagephys2px_x, input_mus_stagephys2px)

    return row_muscle, col_muscle


@nb.jit(nopython=True, parallel=True, nogil=True, fastmath=True)
def map_muscle_to_behavior_jit(
    muscle_image,
    behavior_image_dim,
    x_stage,
    y_stage,
    w_mus_stagephys2px_x,
    w_mus_stagephys2px_y,
    w_beh_stagepx2phys_x,
    w_beh_stagepx2phys_y,
):
    mapped_muscle_image = np.full(behavior_image_dim, np.nan, dtype=muscle_image.dtype)
    for i in nb.prange(behavior_image_dim[0]):
        for j in range(behavior_image_dim[1]):
            muscle_i, muscle_j = get_muscle_px_coords_jit(
                x_stage,
                y_stage,
                i,
                j,
                w_mus_stagephys2px_x,
                w_mus_stagephys2px_y,
                w_beh_stagepx2phys_x,
                w_beh_stagepx2phys_y,
            )
            muscle_i = int(muscle_i)
            muscle_j = int(muscle_j)
            if (
                0 <= muscle_i < muscle_image.shape[0]
                and 0 <= muscle_j < muscle_image.shape[1]
            ):
                mapped_muscle_image[i, j] = muscle_image[muscle_i, muscle_j]
    return mapped_muscle_image


def process_muscle_data(
    recording_dir: Path,
    stage_positions_at_behavior_frames: pd.DataFrame,
    overwrite: bool,
) -> None:
    raw_muscle_images_dir = recording_dir / "muscle_images"
    processed_muscle_images_dir = recording_dir / "processed/muscle_images"
    if processed_muscle_images_dir.is_dir() and not overwrite:
        logging.error(
            f"Output directory (processed muscle images) directory "
            f"'{processed_muscle_images_dir}' already exists. "
            f"Remove it or set overwrite to True."
        )
        return
    processed_muscle_images_dir.mkdir(exist_ok=True, parents=True)
    behavior_calibration_path = (
        recording_dir / "metadata/calibration_parameters_behavior.yaml"
    )
    muscle_calibration_path = (
        recording_dir / "metadata/calibration_parameters_muscle.yaml"
    )

    experiment_params_path = recording_dir / "metadata/experiment_parameters.yaml"
    with open(experiment_params_path, "r") as f:
        experiment_params = yaml.safe_load(f)
    muscle_sync_ratio = experiment_params["muscle_sync_ratio"]
    stage_positions_at_muscle_frames = stage_positions_at_behavior_frames[
        ::muscle_sync_ratio
    ]

    recorder_config_path = recording_dir / "metadata/recorder_config.yaml"
    with open(recorder_config_path, "r") as f:
        recorder_config = yaml.safe_load(f)
    # Attention! The recorder config file specifies ROI dimensions as
    # defined on the physical sensor. The acquisition software flips the
    # image and rotates it by 90 degrees in order to keep it consistent
    # with the fly arena orientation. Here the dimension should be the
    # dimension of the reoriented image. Therefore, nrow is roi_width
    # from the recorder config and ncol is roi_height.
    behavior_image_dim = (
        recorder_config["behavior_camera"]["roi_width"],
        recorder_config["behavior_camera"]["roi_height"],
    )

    mapping = MuscleToBehaviorMapping(
        behavior_calibration_path,
        muscle_calibration_path,
        behavior_image_dim,
    )

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
    for frame_idx in range(stage_positions_at_muscle_frames.shape[0]):
        if frame_idx not in _muscle_paths_by_frame_idx:
            logging.error(
                f"Problem scanning muscle images: Frame {frame_idx} not found in "
                f"{raw_muscle_images_dir}. Dataset is incomplete."
            )
            raise RuntimeError("Dataset is incomplete.")
        muscle_image_paths.append(_muscle_paths_by_frame_idx[frame_idx])

    # Process each muscle image
    print("Warping muscle images...")
    for i, in_path in tqdm(
        enumerate(muscle_image_paths), total=len(muscle_image_paths), disable=None
    ):
        in_image = cv2.imread(str(in_path), cv2.IMREAD_UNCHANGED)
        stage_pos_log_entry = stage_positions_at_muscle_frames.iloc[i]
        x_stage = stage_pos_log_entry["x_pos_mm_interp"]
        y_stage = stage_pos_log_entry["y_pos_mm_interp"]
        out_image = mapping.warp_muscle_image(in_image, x_stage, y_stage)
        out_path = processed_muscle_images_dir / in_path.name
        cv2.imwrite(str(out_path), out_image, _imwrite_compression_params)
