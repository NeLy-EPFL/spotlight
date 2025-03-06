import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
import seaborn as sns
from itertools import product
from scipy.interpolate import interp1d
from pathlib import Path
from tqdm import tqdm
from joblib import Parallel, delayed
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.metrics import mean_squared_error, r2_score

from calibration.resample_images import (
    make_spatial_grid,
    resample_spatial_grid,
    resample_temporal_grid,
    interpolate_stage_position,
    add_interpolated_positions,
    find_keyframes,
)
from calibration.aruco import ArUcoBoard, detect_aruco, plot_aruco_detections
from calibration.model import (
    fit_affine_mapping,
    predict_physical_coords,
    visualize_calibration_results,
    ransac_filter_outliers,
    visualize_ransac_results,
)

aruco_board = ArUcoBoard(arena_dim_mm=(48, 72), scale_mm=0.3)


def index_images_metadata(aruco_scan_dir):
    # Scan image metadata
    behavior_image_files = sorted(
        list((aruco_scan_dir / "behavior_images/").glob("*.csv"))
    )

    _individual_dfs = []
    for path in behavior_image_files:
        _df = pd.read_csv(path)
        assert len(_df) == 3
        _df.loc[:, "path"] = str(path).replace(".csv", ".jpg")
        _df.loc[:, "channel"] = [0, 1, 2]
        _individual_dfs.append(_df)
    return pd.concat(_individual_dfs, ignore_index=True)


def load_keyframe_images(keyframe_metadata_df):
    keyframe_images = {}
    for path in keyframe_metadata_df["path"].unique():
        sel = keyframe_metadata_df[keyframe_metadata_df["path"] == path]
        image_all_channels = plt.imread(path)
        for _, row in sel.iterrows():
            channel = row["channel"]
            keyframe_images[row["frameId"]] = image_all_channels[:, :, channel]
    return keyframe_images


def extract_coordinates_dataframe(
    keyframe_metadata_df, aruco_detection_results, aruco_board
):
    data = {
        "stage_x_mm": [],
        "stage_y_mm": [],
        "pixel_x_px": [],
        "pixel_y_px": [],
        "physical_x_mm": [],
        "physical_y_mm": [],
    }
    for _, row in keyframe_metadata_df.iterrows():
        frame_id = row["frameId"]
        stage_x_mm = row["stage_pos_x_mm"]
        stage_y_mm = row["stage_pos_y_mm"]

        detection_res = aruco_detection_results[frame_id]
        for i, aruco_id in enumerate(detection_res["ids"]):
            physical_corners_pos = aruco_board.grid_id_to_corner_xy_mm(aruco_id)
            for j in range(4):
                pixel_x, pixel_y = detection_res["corners"][i, j, :]
                physical_x, physical_y = physical_corners_pos[j, :]
                data["stage_x_mm"].append(stage_x_mm)
                data["stage_y_mm"].append(stage_y_mm)
                data["pixel_x_px"].append(pixel_x)
                data["pixel_y_px"].append(pixel_y)
                data["physical_x_mm"].append(physical_x)
                data["physical_y_mm"].append(physical_y)

    return pd.DataFrame(data)


def run_calibration(base_dir, aruco_board, grid_stride=2):
    # Load stage position and image metadata
    images_metadata_df = index_images_metadata(base_dir)
    stage_position_log = pd.read_csv(base_dir / "stage_position/stage_position.csv")

    # Resample spatially and temporally; find keyframes
    keyframe_metadata_df = find_keyframes(
        stage_position_log, images_metadata_df, grid_stride
    )

    # Load keyframes and detect codes
    keyframe_images = load_keyframe_images(keyframe_metadata_df)
    aruco_detection_results = {}
    for _, row in tqdm(
        keyframe_metadata_df.iterrows(), total=len(keyframe_metadata_df)
    ):
        frame_id = row["frameId"]
        image = keyframe_images[frame_id]
        ids, corners = detect_aruco(image, horizontal_flip=True)
        aruco_detection_results[frame_id] = {"ids": ids, "corners": corners}

    # Extract coordinates dataframe
    coordinates_df = extract_coordinates_dataframe(
        keyframe_metadata_df, aruco_detection_results, aruco_board
    )

    # # Fit affine mapping
    # calibration_model = fit_affine_mapping(coordinates_df)
    ransac_result = ransac_filter_outliers(coordinates_df)
    models = {"x": ransac_result["models"]["x"], "y": ransac_result["models"]["y"]}

    return models, ransac_result, coordinates_df


def A_to_B(A):
    M = A[:, 2:4]  # row and column weights
    M_inv = np.linalg.inv(M)
    A_stage = A[:, 0:2]  # stage position
    A_bias = A[:, 4:5]  # bias
    B_stage = -M_inv @ A_stage
    B_physical = M_inv
    B_bias = -M_inv @ A_bias
    B = np.hstack([B_stage, B_physical, B_bias])
    return B


if __name__ == "__main__":
    models_rbr, ransac_result_rbr, coordinates_df_rbr = run_calibration(
        Path.home() / "Spotlight/calibration/aruco_scan/row_by_row", aruco_board
    )
    models_cbc, ransac_result_cbc, coordinates_df_cbc = run_calibration(
        Path.home() / "Spotlight/calibration/aruco_scan/column_by_column", aruco_board
    )
    visualize_ransac_results(coordinates_df_rbr, ransac_result_rbr)
    visualize_ransac_results(coordinates_df_cbc, ransac_result_cbc)

    y_model = ransac_result_rbr["models"]["y"]
    x_model = ransac_result_cbc["models"]["x"]

    mat_pixel_to_physical = np.array(
        [
            [*x_model.coef_.squeeze(), x_model.intercept_],
            [*y_model.coef_.squeeze(), y_model.intercept_],
        ]
    )

    mat_physical_to_pixel = A_to_B(mat_pixel_to_physical)

    params_str_lines = [
        f"physicalPosX_wStagePosX {mat_pixel_to_physical[0, 0]}",
        f"physicalPosX_wStagePosY {mat_pixel_to_physical[0, 1]}",
        f"physicalPosX_wPixelPosRow {mat_pixel_to_physical[0, 2]}",
        f"physicalPosX_wPixelPosCol {mat_pixel_to_physical[0, 3]}",
        f"physicalPosX_bias {mat_pixel_to_physical[0, 4]}",
        f"physicalPosY_wStagePosX {mat_pixel_to_physical[1, 0]}",
        f"physicalPosY_wStagePosY {mat_pixel_to_physical[1, 1]}",
        f"physicalPosY_wPixelPosRow {mat_pixel_to_physical[1, 2]}",
        f"physicalPosY_wPixelPosCol {mat_pixel_to_physical[1, 3]}",
        f"physicalPosY_bias {mat_pixel_to_physical[1, 4]}",

        f"pixelPosRow_wStagePosX {mat_physical_to_pixel[0, 0]}",
        f"pixelPosRow_wStagePosY {mat_physical_to_pixel[0, 1]}",
        f"pixelPosRow_wPhysicalPosX {mat_physical_to_pixel[0, 2]}",
        f"pixelPosRow_wPhysicalPosY {mat_physical_to_pixel[0, 3]}",
        f"pixelPosRow_bias {mat_physical_to_pixel[0, 4]}",
        f"pixelPosCol_wStagePosX {mat_physical_to_pixel[1, 0]}",
        f"pixelPosCol_wStagePosY {mat_physical_to_pixel[1, 1]}",
        f"pixelPosCol_wPhysicalPosX {mat_physical_to_pixel[1, 2]}",
        f"pixelPosCol_wPhysicalPosY {mat_physical_to_pixel[1, 3]}",
        f"pixelPosCol_bias {mat_physical_to_pixel[1, 4]}",
    ]

    calibration_result_path = Path.home() / "Spotlight/calibration/calibration_result.txt"
    with open(calibration_result_path, "w") as f:
        f.write("\n".join(params_str_lines))