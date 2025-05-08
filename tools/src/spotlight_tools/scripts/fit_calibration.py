import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml
import cv2
import tyro
from pathlib import Path
from tqdm import tqdm

import spotlight_tools.file_format_versions as versions
from spotlight_tools.calibration.aruco import (
    ArUcoBoard,
    detect_aruco,
    plot_aruco_detections,
)
from spotlight_tools.calibration.model import (
    ransac_filter_outliers,
    visualize_ransac_results,
)


def gather_calibration_points(
    aruco_board: ArUcoBoard,
    calibration_image_dir: Path,
    aruco_detection_viz_dir: Path | None = None,
) -> pd.DataFrame:
    all_data = []

    for path in tqdm(list(calibration_image_dir.glob("*.jpg"))):
        _parts = path.stem.split("_")
        physical_x_mm = float(_parts[2].replace("x", ""))
        physical_y_mm = float(_parts[3].replace("y", ""))
        image = cv2.imread(str(path))
        assert len(image.shape) == 3
        image = image[:, :, 0]
        ids, corners = detect_aruco(image, horizontal_flip=False)
        if aruco_detection_viz_dir is not None:
            fig, ax = plt.subplots()
            plot_aruco_detections(fig, ax, image, ids, corners)
            fig.savefig(aruco_detection_viz_dir / f"{path.stem}.png")
            plt.close(fig)
        if ids is None:
            continue
        for i, aruco_id in enumerate(ids):
            physical_corners_pos = aruco_board.grid_id_to_corner_xy_mm(aruco_id)
            for j in range(4):
                data = {
                    "aruco_id": aruco_id,
                    "stage_x_mm": physical_x_mm,
                    "stage_y_mm": physical_y_mm,
                    "pixel_x_px": corners[i][j][0],
                    "pixel_y_px": corners[i][j][1],
                    "physical_x_mm": physical_corners_pos[j][0],
                    "physical_y_mm": physical_corners_pos[j][1],
                    "image_path": str(path),
                    "corner_id": j,
                }
                all_data.append(data)

    return pd.DataFrame(all_data)


def A_to_B(A):
    M = A[:, 2:4]  # pixel x and y weights
    M_inv = np.linalg.inv(M)
    A_stage = A[:, 0:2]  # stage position
    A_bias = A[:, 4:5]  # bias
    B_stage = -M_inv @ A_stage
    B_physical = M_inv
    B_bias = -M_inv @ A_bias
    B = np.hstack([B_stage, B_physical, B_bias])
    return B


def fit_calibration_model(
    profile_dir: str = "~/Spotlight/default/",
    arena_width: float = 48,
    arena_height: float = 72,
    aruco_scale_mm: float = 0.3,
    aruco_spacing_unitblk: int = 2,
    visualize_aruco_detections: bool = False,
) -> None:
    """
    Fit the calibration model for the ArUco board.

    Args:
        profile_dir (str):
            Path to the profile directory.
        arena_width (float):
            Width of the arena in mm.
        arena_height (float):
            Height of the arena in mm.
        aruco_scale_mm (float):
            Size of each "pixel" (i.e. "block") of the ArUco markers in mm.
        aruco_spacing_unitblk (int):
            Spacing between each ArUco marker in "pixel" (i.e. "block").
        visualize_aruco_detections (bool):
            Whether to visualize the ArUco detections.
    """
    # Expand the profile directory
    profile_dir = Path(profile_dir).expanduser()

    # Check if the calibration images directory exists and is not empty
    calibration_image_dir = Path(profile_dir) / "calibration/aruco_scan/behavior_camera"
    if not calibration_image_dir.exists():
        raise FileNotFoundError(
            f"Directory {calibration_image_dir} does not exist. "
            "Run calibration scan first."
        )
    if not list(calibration_image_dir.glob("*.jpg")):
        raise FileNotFoundError(
            f"Directory {calibration_image_dir} is empty. "
            "Run calibration scan first."
        )

    # Create the directory for visualizing ArUco detections if necessary
    aruco_detection_viz_dir = Path(profile_dir) / "calibration/aruco_scan_detection"
    if visualize_aruco_detections:
        aruco_detection_viz_dir.mkdir(exist_ok=True)

    # Initialize the ArUco board
    aruco_board = ArUcoBoard(
        arena_dim_mm=(arena_width, arena_height),
        scale_mm=aruco_scale_mm,
        spacing_unitblk=aruco_spacing_unitblk,
    )

    # Gather calibration points
    coordinates_df = gather_calibration_points(
        aruco_board,
        calibration_image_dir,
        aruco_detection_viz_dir if visualize_aruco_detections else None,
    )
    coordinates_df_path = Path(profile_dir) / "calibration/calibration_points.csv"
    coordinates_df.to_csv(coordinates_df_path, index=False)

    # Fit model
    ransac_result = ransac_filter_outliers(coordinates_df)
    visualize_ransac_results(coordinates_df, ransac_result)

    x_model = ransac_result["models"]["x"]
    y_model = ransac_result["models"]["y"]

    mat_pixel_to_physical = np.array(
        [
            [*x_model.coef_.squeeze(), x_model.intercept_],
            [*y_model.coef_.squeeze(), y_model.intercept_],
        ]
    )

    mat_physical_to_pixel = A_to_B(mat_pixel_to_physical)

    calibration_results = {
        "metadata": {
            # Assign a version number to the format of this calibration file
            # Follow semver for backward compatibility detection
            "file_format_version": {
                "major": versions.calibration_result_major,
                "minor": versions.calibration_result_minor,
                "patch": versions.calibration_result_patch,
            }
        },
        "stage_and_pixel_to_physical": {
            "physical_pos_x": {
                "stage_pos_x": float(mat_pixel_to_physical[0, 0]),
                "stage_pos_y": float(mat_pixel_to_physical[0, 1]),
                "pixel_pos_x": float(mat_pixel_to_physical[0, 2]),
                "pixel_pos_y": float(mat_pixel_to_physical[0, 3]),
                "bias": float(mat_pixel_to_physical[0, 4]),
            },
            "physical_pos_y": {
                "stage_pos_x": float(mat_pixel_to_physical[1, 0]),
                "stage_pos_y": float(mat_pixel_to_physical[1, 1]),
                "pixel_pos_x": float(mat_pixel_to_physical[1, 2]),
                "pixel_pos_y": float(mat_pixel_to_physical[1, 3]),
                "bias": float(mat_pixel_to_physical[1, 4]),
            },
        },
        "stage_and_physical_to_pixel": {
            "pixel_pos_x": {
                "stage_pos_x": float(mat_physical_to_pixel[0, 0]),
                "stage_pos_y": float(mat_physical_to_pixel[0, 1]),
                "physical_pos_x": float(mat_physical_to_pixel[0, 2]),
                "physical_pos_y": float(mat_physical_to_pixel[0, 3]),
                "bias": float(mat_physical_to_pixel[0, 4]),
            },
            "pixel_pos_y": {
                "stage_pos_x": float(mat_physical_to_pixel[1, 0]),
                "stage_pos_y": float(mat_physical_to_pixel[1, 1]),
                "physical_pos_x": float(mat_physical_to_pixel[1, 2]),
                "physical_pos_y": float(mat_physical_to_pixel[1, 3]),
                "bias": float(mat_physical_to_pixel[1, 4]),
            },
        },
    }

    calibration_results_path = Path(profile_dir) / "calibration/calibration_result.yaml"
    with open(calibration_results_path, "w") as f:
        yaml.dump(calibration_results, f)


def main():
    tyro.cli(fit_calibration_model)


if __name__ == "__main__":
    main()
