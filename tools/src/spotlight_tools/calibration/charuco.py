import cv2
import numpy as np
from typing import Tuple, Optional


class CharucoBoard:
    """Class to handle ChArUco board creation and detection."""

    def __init__(
        self,
        squares_x: int = 7,
        squares_y: int = 6,
        square_length: float = 0.0005,
        marker_length: float = 0.000375,
        aruco_dict: int = cv2.aruco.DICT_4X4_250,
        h_flip: bool = True,
    ):
        """
        Initialize a ChArUco board.

        Args:
            squares_x: Number of squares in X direction
            squares_y: Number of squares in Y direction
            square_length: Square side length (in pixels)
            marker_length: Marker side length (in pixels)
            aruco_dict: ArUco dictionary type
        """
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_length = square_length
        self.marker_length = marker_length

        self.h_flip = h_flip

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, self.aruco_dict
        )
        self.board.setLegacyPattern(True)

    def detect_corners(
        self,
        image: np.ndarray,
        camera: str = "behavior_camera",
        subpixel_refinement: bool = True,
    ) -> Tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[np.ndarray],
    ]:
        """
        Detect ChArUco corners in an image.

        Args:
            image: Grayscale image
            camera: Camera type for detection parameters
            subpixel_refinement: Whether to perform subpixel corner refinement

        Returns:
            Tuple of (charuco_corners, charuco_ids, marker_corners, marker_ids)
        """
        # Store original image width before flipping
        original_width = image.shape[1]

        # Get detection parameters based on camera type
        detector_params = self._get_detector_params(camera)
        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.minMarkers = 0
        charuco_params.tryRefineMarkers = True

        # Create detector
        detector = cv2.aruco.CharucoDetector(
            self.board, charucoParams=charuco_params, detectorParams=detector_params
        )

        # Flip image if needed for better detection
        working_image = image
        if self.h_flip:
            working_image = image[:, ::-1].copy()

        # Detect board
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(
            working_image
        )

        # Subpixel refinement on the working image
        if subpixel_refinement and charuco_ids is not None and len(charuco_ids) > 0:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            cv2.cornerSubPix(
                working_image,
                charuco_corners,
                winSize=(5, 5),
                zeroZone=(-1, -1),
                criteria=criteria,
            )

        # Flip coordinates back to original image space
        if self.h_flip and charuco_corners is not None and len(charuco_corners) > 0:
            # ChArUco corners are shape (N, 1, 2) where [:, :, 0] is x-coordinate
            charuco_corners[:, :, 0] = original_width - 1 - charuco_corners[:, :, 0]

            # Marker corners are a list of arrays, each shape (1, 4, 2)
            if marker_corners is not None:
                for i in range(len(marker_corners)):
                    marker_corners[i][:, :, 0] = (
                        original_width - 1 - marker_corners[i][:, :, 0]
                    )

        return charuco_corners, charuco_ids, marker_corners, marker_ids

    def _get_detector_params(self, camera: str) -> cv2.aruco.DetectorParameters:
        """Get ArUco detection parameters for different cameras."""
        if camera == "behavior_camera":
            parameters = cv2.aruco.DetectorParameters()
            return parameters
        elif camera == "muscle_camera":
            parameters = cv2.aruco.DetectorParameters()
            # parameters.adaptiveThreshWinSizeMin = 3
            # parameters.adaptiveThreshWinSizeMax = 53
            # parameters.adaptiveThreshWinSizeStep = 4

            # parameters.adaptiveThreshConstant = 1

            # parameters.minMarkerPerimeterRate = 0.005
            # parameters.maxMarkerPerimeterRate = 10.0

            # parameters.minCornerDistanceRate = 0.01
            # parameters.minDistanceToBorder = 0
            return parameters
        else:
            raise ValueError(f"Unknown camera type: {camera}")


def get_gizem_board() -> CharucoBoard:
    """Get the default ChArUco board configuration."""
    return CharucoBoard(
        squares_x=7,
        squares_y=6,
        square_length=300,
        marker_length=225,
        aruco_dict=cv2.aruco.DICT_4X4_250,
    )
