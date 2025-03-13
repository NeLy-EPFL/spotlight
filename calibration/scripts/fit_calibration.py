import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml
import cv2
from pathlib import Path
from tqdm import tqdm

from calibration.aruco import ArUcoBoard, detect_aruco, plot_aruco_detections
from calibration.model import ransac_filter_outliers, visualize_ransac_results


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


aruco_board = ArUcoBoard(arena_dim_mm=(48, 72), scale_mm=0.3, spacing_unitblk=2)

calibration_image_dir = Path.home() / "Spotlight/calibration/aruco_scan"
aruco_detection_viz_dir = Path.home() / "Spotlight/calibration/aruco_scan_detection"
aruco_detection_viz_dir.mkdir(exist_ok=True)

all_data = []
for path in tqdm(list(calibration_image_dir.glob("*.jpg"))):
    _parts = path.stem.split("_")
    physical_x_mm = float(_parts[2].replace("x", ""))
    physical_y_mm = float(_parts[3].replace("y", ""))
    image = cv2.imread(str(path))
    assert len(image.shape) == 3
    image = image[:, :, 0]
    # image = cv2.fastNlMeansDenoising(image, None, 20, 7, 21)
    # image_ = image.copy()
    # image = np.ones_like(image) * 180
    # image[:int(image.shape[0] / 2), :int(image.shape[1] / 2)] = image_[::2, ::2]
    # image[image < 128] = 0
    # image[image >= 128] = 255
    ids, corners = detect_aruco(image, horizontal_flip=False)
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
coordinates_df = pd.DataFrame(all_data)

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
    "stage_and_pixel_to_physical": {
        "physical_pos_x": {
            "stage_pos_x": float(mat_pixel_to_physical[0, 0]),
            "stage_pos_y": float(mat_pixel_to_physical[0, 1]),
            "pixel_pos_row": float(mat_pixel_to_physical[0, 2]),
            "pixel_pos_col": float(mat_pixel_to_physical[0, 3]),
            "bias": float(mat_pixel_to_physical[0, 4]),
        },
        "physical_pos_y": {
            "stage_pos_x": float(mat_pixel_to_physical[1, 0]),
            "stage_pos_y": float(mat_pixel_to_physical[1, 1]),
            "pixel_pos_row": float(mat_pixel_to_physical[1, 2]),
            "pixel_pos_col": float(mat_pixel_to_physical[1, 3]),
            "bias": float(mat_pixel_to_physical[1, 4]),
        },
    },
    "stage_and_physical_to_pixel": {
        "pixel_pos_row": {
            "stage_pos_x": float(mat_physical_to_pixel[0, 0]),
            "stage_pos_y": float(mat_physical_to_pixel[0, 1]),
            "physical_pos_x": float(mat_physical_to_pixel[0, 2]),
            "physical_pos_y": float(mat_physical_to_pixel[0, 3]),
            "bias": float(mat_physical_to_pixel[0, 4]),
        },
        "pixel_pos_col": {
            "stage_pos_x": float(mat_physical_to_pixel[1, 0]),
            "stage_pos_y": float(mat_physical_to_pixel[1, 1]),
            "physical_pos_x": float(mat_physical_to_pixel[1, 2]),
            "physical_pos_y": float(mat_physical_to_pixel[1, 3]),
            "bias": float(mat_physical_to_pixel[1, 4]),
        },
    },
}

coordinates_df.to_csv(
    Path.home() / "Spotlight/calibration/calibration_points.csv", index=False
)

calibration_results_path = Path.home() / "Spotlight/calibration/calibration_result.yaml"
with open(calibration_results_path, "w") as f:
    yaml.dump(calibration_results, f)
