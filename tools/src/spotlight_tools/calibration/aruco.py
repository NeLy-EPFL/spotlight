import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# These parameters very empirically tuned for the two cameras
# See https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
# for all parameters.
# See https://docs.opencv.org/4.11.0/d5/dae/tutorial_aruco_detection.html
# (Detector Parameters section) for a walkthrough. However, note that this tutorial
# doesn't cover all parameters.
_detection_params_behavior_cam = cv2.aruco.DetectorParameters()
_detection_params_behavior_cam.perspectiveRemovePixelPerCell = 40
_detection_params_behavior_cam.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

_detection_params_muscle_cam = cv2.aruco.DetectorParameters()
_detection_params_muscle_cam.minMarkerPerimeterRate = 1.5
_detection_params_muscle_cam.maxMarkerPerimeterRate = 6.0
_detection_params_muscle_cam.adaptiveThreshWinSizeMax = 46
_detection_params_muscle_cam.perspectiveRemovePixelPerCell = 40
_detection_params_muscle_cam.adaptiveThreshWinSizeMin = 50
_detection_params_muscle_cam.adaptiveThreshWinSizeMax = 120
_detection_params_muscle_cam.adaptiveThreshConstant = 20
_detection_params_muscle_cam.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

aruco_detection_params = {
    "behavior_camera": _detection_params_behavior_cam,
    "muscle_camera": _detection_params_muscle_cam,
}
aruco_detection_image_preprocessing = {
    "behavior_camera": lambda x: x.copy(),  # make a copy to avoid modifying the original
    "muscle_camera": lambda x: cv2.GaussianBlur(x, (5, 5), 0),
}


def detect_aruco(
    image, camera, horizontal_flip=False, dictionary=cv2.aruco.DICT_4X4_1000
):
    """
    Detect ArUco codes in an image and return their IDs and corner coordinates.

    Parameters:
    -----------
    image : numpy.ndarray
        Grayscale image of shape (rows, cols)
    camera : str
        "behavior_camera" or "muscle_camera".
    horizontal_flip : bool, optional
        Whether to flip the image horizontally before detection (default: False)
    dictionary : cv2.aruco.Dictionary, optional
        ArUco dictionary to use for detection (default: cv2.aruco.DICT_4X4_1000)

    Returns:
    --------
    tuple
        (ids, coords) where:
        - ids is an array of shape (n,) containing the ArUco IDs
        - coords is an array of shape (n, 4, 2) containing the corner coordinates
          in pixel units, with each corner having (x, y) coordinates
    """
    if camera not in ("behavior_camera", "muscle_camera"):
        raise ValueError("camera must be either 'behavior_camera' or 'muscle_camera'.")

    # Preprocess image
    working_image = aruco_detection_image_preprocessing[camera](image)

    # Get image dimensions
    num_rows, num_cols = working_image.shape

    # Flip the image horizontally if requested
    if horizontal_flip:
        working_image = cv2.flip(working_image, 1)  # 1 means horizontal flip

    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)

    # Detect ArUco markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_detection_params[camera])
    corners, ids, rejected = detector.detectMarkers(working_image)

    # If no markers are detected, return empty arrays
    if ids is None:
        return np.array([]), np.zeros((0, 4, 2))

    # Convert corners to proper format
    coords = np.array([corner.reshape(4, 2) for corner in corners])

    # Flip coordinates back if the image was flipped
    if horizontal_flip:
        # For each detected marker
        for i in range(coords.shape[0]):
            # For each corner point
            for j in range(4):
                # Flip the x-coordinate
                coords[i, j, 0] = num_cols - coords[i, j, 0] - 1

    return ids.flatten(), coords


def plot_aruco_detections(fig, ax, image, ids, coords):
    """
    Plot the detected ArUco codes on the image with different colors for each corner.

    Parameters:
    -----------
    fig : matplotlib.figure.Figure
        Figure object to plot on
    ax : matplotlib.axes.Axes
        Axes object to plot on
    image : numpy.ndarray
        Original grayscale image
    ids : numpy.ndarray
        Array of ArUco IDs of shape (n,)
    coords : numpy.ndarray
        Array of corner coordinates of shape (n, 4, 2)
    """
    # Display the image
    ax.imshow(image, cmap="gray")

    # Colors for the four corners
    corner_colors = ["magenta", "orange", "blue", "cyan"]

    # For each detected ArUco marker
    for i in range(len(ids)):
        marker_id = ids[i]
        corners = coords[i]

        # Plot each corner with a different color and add a small marker
        for j in range(4):
            x, y = corners[j]
            ax.plot(
                x, y, "x", color=corner_colors[j], markersize=4, label=f"Corner {j}"
            )
        if i == 0:
            ax.legend(
                loc="upper center", bbox_to_anchor=(0.5, -0.05), frameon=False, ncol=4
            )

        # Connect the corners with lines to form the square
        # Add the last line connecting the last and first corners to close the polygon
        corners_closed = np.vstack([corners, corners[0]])
        ax.plot(
            corners_closed[:, 0],
            corners_closed[:, 1],
            "-",
            color="yellow",
            linewidth=1,
        )

        # Add the ID text at the center of the marker
        center_x = np.mean(corners[:, 0])
        center_y = np.mean(corners[:, 1])
        ax.text(
            center_x,
            center_y,
            str(marker_id),
            color="white",
            fontsize=8,
            ha="center",
            va="center",
            bbox=dict(facecolor="black", alpha=0.3, pad=2),
        )

    # Set title and labels
    ax.set_title(f"Detected {len(ids)} ArUco Markers")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")

    plt.tight_layout()


import cv2
import numpy as np
import svgwrite


class ArUcoBoard:
    code_size_unitblk = 6

    def __init__(self, arena_dim_mm, scale_mm, spacing_unitblk):
        self.arena_dim_mm = arena_dim_mm  # (width, height)
        self.scale_mm = scale_mm
        self.spacing_unitblk = spacing_unitblk
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

        # Calculate how many codes fit in each dimension
        self.grid_dim_ncodes = (
            int(
                self.arena_dim_mm[0]
                / (self.scale_mm * (self.code_size_unitblk + spacing_unitblk))
            ),
            int(
                self.arena_dim_mm[1]
                / (self.scale_mm * (self.code_size_unitblk + spacing_unitblk))
            ),
        )

        # Check if we have enough markers
        ncodes_total = self.grid_dim_ncodes[0] * self.grid_dim_ncodes[1]
        if ncodes_total > self.aruco_dict.bytesList.shape[0]:
            raise ValueError(
                "Not enough markers in ArUco dictionary for the given arena size"
            )

        # Calculate grid dimensions without margins
        self.grid_dim_no_margin_unitblk = (
            self.grid_dim_ncodes[0] * (self.code_size_unitblk + spacing_unitblk)
            - spacing_unitblk,
            self.grid_dim_ncodes[1] * (self.code_size_unitblk + spacing_unitblk)
            - spacing_unitblk,
        )

        # Calculate margins to center the grid
        self.margins = (
            (self.arena_dim_mm[0] - self.grid_dim_no_margin_unitblk[0] * self.scale_mm)
            / 2,
            (self.arena_dim_mm[1] - self.grid_dim_no_margin_unitblk[1] * self.scale_mm)
            / 2,
        )

    def _grid_id_to_top_left_xy_mm(self, grid_id):
        # Convert grid ID to row and column
        row = grid_id // self.grid_dim_ncodes[0]
        col = grid_id % self.grid_dim_ncodes[0]

        # Calculate x, y coordinates in mm
        x_mm = (
            self.margins[0]
            + col * (self.code_size_unitblk + self.spacing_unitblk) * self.scale_mm
        )
        y_mm = (
            self.margins[1]
            + row * (self.code_size_unitblk + self.spacing_unitblk) * self.scale_mm
        )

        return (x_mm, y_mm)

    def grid_id_to_corner_xy_mm(self, grid_id):
        x_left, y_top = self._grid_id_to_top_left_xy_mm(grid_id)
        x_right = x_left + self.code_size_unitblk * self.scale_mm
        y_bottom = y_top + self.code_size_unitblk * self.scale_mm
        return np.array(
            [[x_left, y_top], [x_right, y_top], [x_right, y_bottom], [x_left, y_bottom]]
        )

    def draw_svg(self, output_path):
        dwg = svgwrite.Drawing(
            output_path,
            profile="tiny",
            size=(
                f"{self.arena_dim_mm[0]}mm",  # width
                f"{self.arena_dim_mm[1]}mm",  # height
            ),
        )

        # Iterate through the grid and generate ArUco codes
        for r in range(self.grid_dim_ncodes[1]):
            for c in range(self.grid_dim_ncodes[0]):
                # Generate a unique ArUco marker ID
                marker_id = r * self.grid_dim_ncodes[0] + c

                # Create the marker image
                marker_img = np.zeros(
                    (self.code_size_unitblk, self.code_size_unitblk), dtype=np.uint8
                )
                cv2.aruco.generateImageMarker(
                    self.aruco_dict, marker_id, self.code_size_unitblk, marker_img, 1
                )

                # Calculate the position of the top-left corner of the marker
                x_offset_mm, y_offset_mm = self._grid_id_to_top_left_xy_mm(marker_id)

                # Add the marker to the SVG as a group of rectangles
                for px_row in range(self.code_size_unitblk):
                    for px_col in range(self.code_size_unitblk):
                        if marker_img[px_row, px_col] == 0:  # Draw black squares
                            dwg.add(
                                dwg.rect(
                                    insert=(
                                        f"{x_offset_mm + px_col * self.scale_mm}mm",
                                        f"{y_offset_mm + px_row * self.scale_mm}mm",
                                    ),
                                    size=(f"{self.scale_mm}mm", f"{self.scale_mm}mm"),
                                    fill="black",
                                )
                            )
        dwg.save()
