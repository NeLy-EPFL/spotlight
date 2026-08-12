import numpy as np
import yaml
import cv2
from pathlib import Path


class SpotlightPositionMapper:
    """
    Maps between stage, physical, and pixel coordinates using calibration
    parameters.

    This class provides methods to convert between:
      - Stage and physical positions to pixel coordinates
        (`stage_and_physical_to_pixel`)
      - Stage and pixel coordinates to physical positions
        (`stage_and_pixel_to_physical`)

    The mapping is parameterized by calibration data, which can be provided
    as a dictionary or as a path to a YAML calibration file.

    Args:
        calibration_parameters (dict | str | Path): Calibration parameters
            as a dictionary, or a path to a YAML file containing the
            calibration data.

    Example:
        >>> mapper = SpotlightPositionMapper("calibration.yaml")
        >>> pixel_coords = mapper.stage_and_physical_to_pixel(stage_pos, physical_pos)
        >>> physical_coords = mapper.stage_and_pixel_to_physical(stage_pos, pixel_coords)
    """

    def __init__(self, calibration_parameters: dict | str | Path):
        if isinstance(calibration_parameters, (str, Path)):
            with open(calibration_parameters, "r") as f:
                calibration_parameters = yaml.safe_load(f)

        # Stage and physical to pixel mapping
        _weights_stage_and_physical_to_pixel_names = [
            "stage_pos_x",
            "stage_pos_y",
            "physical_pos_x",
            "physical_pos_y",
            "bias",
        ]
        stage_and_physical_to_pixel_params_dict = calibration_parameters[
            "stage_and_physical_to_pixel"
        ]
        self._weights_stage_and_physical_to_pixel_x = np.array(
            [
                stage_and_physical_to_pixel_params_dict["pixel_pos_x"][key]
                for key in _weights_stage_and_physical_to_pixel_names
            ]
        )
        self._weights_stage_and_physical_to_pixel_y = np.array(
            [
                stage_and_physical_to_pixel_params_dict["pixel_pos_y"][key]
                for key in _weights_stage_and_physical_to_pixel_names
            ]
        )

        # Stage and pixel to physical mapping
        self._weights_stage_and_pixel_to_physical_names = [
            "stage_pos_x",
            "stage_pos_y",
            "pixel_pos_x",
            "pixel_pos_y",
            "bias",
        ]
        stage_and_pixel_to_physical_params_dict = calibration_parameters[
            "stage_and_pixel_to_physical"
        ]
        self._weights_stage_and_pixel_to_physical_x = np.array(
            [
                stage_and_pixel_to_physical_params_dict["physical_pos_x"][key]
                for key in self._weights_stage_and_pixel_to_physical_names
            ]
        )
        self._weights_stage_and_pixel_to_physical_y = np.array(
            [
                stage_and_pixel_to_physical_params_dict["physical_pos_y"][key]
                for key in self._weights_stage_and_pixel_to_physical_names
            ]
        )

    def stage_and_physical_to_pixel(
        self, stage_pos: np.ndarray, physical_pos: np.ndarray
    ):
        """
        Convert stage and physical positions to pixel coordinates.

        Args:
            stage_pos: Stage position as a numpy array of shape (*, 2),
                where the last dimension contains (x, y) coordinates.
            physical_pos: Physical position as a numpy array. The shape
                should match stage_pos, with the last dimension being
                (x, y) coordinates.

        Returns:
            Pixel coordinates as a numpy array of the same shape as
            stage_pos and physical_pos, with the last dimension being
            pixel (x, y) coordinates (i.e. column, row).
        """
        if stage_pos.shape[-1] != 2 or physical_pos.shape[-1] != 2:
            raise ValueError(
                "stage_pos and physical_pos must have the last dimension of size 2"
            )

        stage_pos_2d = stage_pos.reshape(-1, 2)
        physical_pos_2d = physical_pos.reshape(-1, 2)
        inputs = np.hstack(
            (
                stage_pos_2d,
                physical_pos_2d,
                np.ones((stage_pos_2d.shape[0], 1)),  # Bias term
            )
        )
        pixel_pos_x = np.dot(inputs, self._weights_stage_and_physical_to_pixel_x)
        pixel_pos_y = np.dot(inputs, self._weights_stage_and_physical_to_pixel_y)
        pixel_pos = np.hstack((pixel_pos_x[:, np.newaxis], pixel_pos_y[:, np.newaxis]))
        return pixel_pos.reshape(stage_pos.shape)

    def stage_and_pixel_to_physical(self, stage_pos: np.ndarray, pixel_pos: np.ndarray):
        """
        Convert stage and pixel coordinates to physical positions.

        Args:
            stage_pos: Stage position as a numpy array of shape (*, 2),
                where the last dimension contains (x, y) coordinates.
            pixel_pos: Pixel position as a numpy array. The shape
                should match stage_pos, with the last dimension being
                (x, y) coordinates.
        Returns:
            Physical coordinates as a numpy array of the same shape as
            stage_pos and pixel_pos, with the last dimension being
            physical (x, y) coordinates.
        """
        if stage_pos.shape[-1] != 2 or pixel_pos.shape[-1] != 2:
            raise ValueError(
                "stage_pos and pixel_pos must have the last dimension of size 2"
            )

        stage_pos_2d = stage_pos.reshape(-1, 2)
        pixel_pos_2d = pixel_pos.reshape(-1, 2)
        inputs = np.hstack(
            (
                stage_pos_2d,
                pixel_pos_2d,
                np.ones((stage_pos_2d.shape[0], 1)),  # Bias term
            )
        )
        physical_pos_x = np.dot(inputs, self._weights_stage_and_pixel_to_physical_x)
        physical_pos_y = np.dot(inputs, self._weights_stage_and_pixel_to_physical_y)
        physical_pos = np.hstack(
            (physical_pos_x[:, np.newaxis], physical_pos_y[:, np.newaxis])
        )
        return physical_pos.reshape(stage_pos.shape)


class BehaviorMuscleCrossMapper:
    """
    Maps pixel coordinates between behavior and muscle cameras at given
    stage positions.

    This method computes the transformation by composing linear transforms:
        1. Camera A pixels -> physical coordinates
        2. Physical coordinates -> Camera B pixels

    Args:
        behavior_mapper: SpotlightPositionMapper for the behavior camera
        muscle_mapper: SpotlightPositionMapper for the muscle camera

    Example:
        Mapping pixel coordinates:
        >>> cross_mapper = BehaviorMuscleCrossMapper(behavior_mapper, muscle_mapper)
        >>> stage_pos = np.array([100.0, 200.0])
        >>> transform_matrix = cross_mapper.get_affine_matrix_behavior_to_muscle(stage_pos)
        >>> behavior_pixels = np.array([[150, 250], [300, 400]])
        >>> transformed_coords = cross_mapper.transform_pixels_behavior_to_muscle(
        ...     behavior_pixels, stage_pos
        ... )

        Transform an entire image using OpenCV:
        >>> import cv2
        >>> M = cross_mapper.get_affine_matrix_behavior_to_muscle(stage_pos)
        >>> behavior_img = cv2.imread('behavior_frame.png')
        >>> muscle_img_warped = cv2.warpAffine(
        ...     behavior_img, M, (width, height), flags=cv2.INTER_LINEAR
        ... )
    """

    def __init__(
        self,
        behavior_mapper: SpotlightPositionMapper,
        muscle_mapper: SpotlightPositionMapper,
    ):
        self.behavior_mapper = behavior_mapper
        self.muscle_mapper = muscle_mapper

    def _get_affine_matrix_a_to_b(
        self,
        mapper_a: SpotlightPositionMapper,
        mapper_b: SpotlightPositionMapper,
        stage_pos: np.ndarray,
    ):
        """
        Compute the affine transformation matrix to map pixel coordinates
        from A to B (each being either behavior or muscle) at a given stage
        position.

        Args:
            mapper_a: SpotlightPositionMapper for source camera
            mapper_b: SpotlightPositionMapper for target camera
            stage_pos: Stage position as a numpy array of shape (2,)

        Returns:
            A 2x3 affine transformation matrix that maps pixel coordinates
            from camera A to camera B.
        """
        if stage_pos.shape[-1] != 2:
            raise ValueError("stage_pos must have the last dimension of size 2")

        # Extract transformation weights
        # Transformation A: pixels_A -> physical
        # [physical_x]   [ ... W_A_st_px_to_phys_x ... ]   [stage_x  ]
        # [physical_y] = [ ... W_A_st_px_to_phys_y ... ] x [stage_y  ]
        #                                                  [pixel_a_x]
        #                                                  [pixel_a_y]
        #                                                  [1        ]
        W_a = np.array(
            [
                mapper_a._weights_stage_and_pixel_to_physical_x,
                mapper_a._weights_stage_and_pixel_to_physical_y,
            ]
        )  # Shape: (2, 5)

        # Transformation B: stage + physical -> pixels_B
        # [pixel_b_x]   [ ... W_B_st_phys_to_px_x ... ]   [stage_x   ]
        # [pixel_b_y] = [ ... W_B_st_phys_to_px_y ... ] x [stage_y   ]
        #                                                 [physical_x]
        #                                                 [physical_y]
        #                                                 [1         ]
        W_b = np.array(
            [
                mapper_b._weights_stage_and_physical_to_pixel_x,
                mapper_b._weights_stage_and_physical_to_pixel_y,
            ]
        )  # Shape: (2, 5)

        # To compose the transformations, we substitute physical coordinates
        # from A into B. Break down the matrices by columns:
        # W_a = [W_a_stage | W_a_pixel    | W_a_bias]
        # W_b = [W_b_stage | W_b_physical | W_b_bias]
        # where W_*_stage: (2,2), W_*_physical:(2,2), W_*_bias:(2,1)
        W_a_stage = W_a[:, 0:2]
        W_a_pixel = W_a[:, 2:4]
        W_a_bias = W_a[:, 4:5]
        W_b_stage = W_b[:, 0:2]
        W_b_physical = W_b[:, 2:4]
        W_b_bias = W_b[:, 4:5]

        # Then, split B pixel coordinates into:
        # 1. Linear part: Contribution from A pixel xy independent of stage position
        # 2. Offset part: Contribution from stage position independent of A pixel xy

        # Linear part: compose W_b_physical on top of W_a_pixel (chain rule)
        # Interpretation: How changes in pixel_a coordinates affect pixel_b coordinates
        # through their impact on the physical coordinates (chain rule)
        linear_px2px = W_b_physical @ W_a_pixel  # (2, 2)

        # Offset part: further split into
        # i.  How changes in pixel stage position's contribution to physical xy affect
        #     B pixel xy via the physical -> pixel transformation in B, PLUS the direct
        #     contribution from stage position in B
        # ii. How the effect of the bias term in A on physical xy then contributes to
        #     B pixel xy via the physical -> pixel transformation in B
        # Each of these is a (2, 1) vector
        stage_pos_vert = stage_pos.reshape(2, 1)
        stage_contribution = (W_b_stage + W_b_physical @ W_a_stage) @ stage_pos_vert
        bias_contribution = W_b_bias + W_b_physical @ W_a_bias
        offset = stage_contribution + bias_contribution

        # Construct the 2x3 affine matrix
        # [M | offset] = [[M[0,0], M[0,1], offset[0]]
        #                 [M[1,0], M[1,1], offset[1]]]
        affine_matrix = np.hstack([linear_px2px, offset])  # (2, 3)

        return affine_matrix

    def get_affine_matrix_behavior2muscle(self, stage_pos: np.ndarray):
        """
        Compute the affine transformation matrix to map pixel coordinates
        from behavior camera to muscle camera at a given stage position.

        Args:
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.

        Returns:
            A 2x3 affine transformation matrix that maps pixel coordinates
            from behavior camera to muscle camera.
        """
        return self._get_affine_matrix_a_to_b(
            self.behavior_mapper, self.muscle_mapper, stage_pos
        )

    def get_affine_matrix_muscle2behavior(self, stage_pos: np.ndarray):
        """
        Compute the affine transformation matrix to map pixel coordinates
        from muscle camera to behavior camera at a given stage position.

        Args:
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.

        Returns:
            A 2x3 affine transformation matrix that maps pixel coordinates
            from muscle camera to behavior camera.
        """
        return self._get_affine_matrix_a_to_b(
            self.muscle_mapper, self.behavior_mapper, stage_pos
        )

    def transform_image_behavior2muscle(
        self,
        stage_pos: np.ndarray,
        behavior_image: np.ndarray,
        output_dim: tuple[int, int],
    ):
        """
        Warp a behavior image to align with the muscle camera frame at a
        given stage position.

        Args:
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.
            behavior_image: Input behavior image as a numpy array.
            output_dim: Tuple (height, width) specifying the dimensions
                of the output muscle-aligned image.

        Returns:
            Warped muscle-aligned image as a numpy array.
        """
        affine_matrix = self.get_affine_matrix_behavior2muscle(stage_pos)
        warped_image = cv2.warpAffine(
            behavior_image,
            affine_matrix,
            output_dim[::-1],  # OpenCV uses (width, height) order
            flags=cv2.INTER_NEAREST,
        )
        return warped_image

    def transform_image_muscle2behavior(
        self,
        stage_pos: np.ndarray,
        muscle_image: np.ndarray,
        output_dim: tuple[int, int],
    ):
        """
        Warp a muscle image to align with the behavior camera frame at a
        given stage position.

        Args:
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.
            muscle_image: Input muscle image as a numpy array.
            output_dim: Tuple (height, width) specifying the dimensions
                of the output behavior-aligned image.

        Returns:
            Warped behavior-aligned image as a numpy array.
        """
        affine_matrix = self.get_affine_matrix_muscle2behavior(stage_pos)
        warped_image = cv2.warpAffine(
            muscle_image,
            affine_matrix,
            output_dim[::-1],  # OpenCV uses (width, height) order
            flags=cv2.INTER_NEAREST,
        )
        return warped_image

    def map_pixel_coords_behavior2muscle(
        self, behavior_pixel_coords: np.ndarray, stage_pos: np.ndarray
    ):
        """
        Transform pixel coordinates from behavior camera to muscle camera
        using the computed affine transformation.

        Args:
            behavior_pixel_coords: Pixel coordinates in behavior camera as
                numpy array of shape (*, 2), where the last dimension
                contains (x, y) coordinates.
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.

        Returns:
            Pixel coordinates in muscle camera coordinate system.
        """
        if behavior_pixel_coords.shape[-1] != 2:
            raise ValueError("behavior_pixels must have the last dimension of size 2")

        original_shape = behavior_pixel_coords.shape
        behavior_pixels_2d = behavior_pixel_coords.reshape(-1, 2)
        affine_matrix = self.get_affine_matrix_behavior2muscle(stage_pos)
        muscle_pixels_2d = (
            affine_matrix[:, :2] @ behavior_pixels_2d.T
        ).T + affine_matrix[:, 2]
        return muscle_pixels_2d.reshape(original_shape)

    def map_pixel_coords_muscle2behavior(
        self, muscle_pixel_coords: np.ndarray, stage_pos: np.ndarray
    ):
        """
        Transform pixel coordinates from muscle camera to behavior camera
        using the computed affine transformation.

        Args:
            muscle_pixel_coords: Pixel coordinates in muscle camera as
                numpy array of shape (*, 2), where the last dimension
                contains (x, y) coordinates.
            stage_pos: Stage position as a numpy array of shape (2,),
                where the last dimension contains (x, y) coordinates.

        Returns:
            Pixel coordinates in behavior camera coordinate system.
        """
        if muscle_pixel_coords.shape[-1] != 2:
            raise ValueError("muscle_pixels must have the last dimension of size 2")

        original_shape = muscle_pixel_coords.shape
        muscle_pixels_2d = muscle_pixel_coords.reshape(-1, 2)
        affine_matrix = self.get_affine_matrix_muscle2behavior(stage_pos)
        behavior_pixels_2d = (
            affine_matrix[:, :2] @ muscle_pixels_2d.T
        ).T + affine_matrix[:, 2]
        return behavior_pixels_2d.reshape(original_shape)

    def _validate_transformation_accuracy(
        self, stage_pos: np.ndarray, test_pixels: np.ndarray = None
    ):
        """
        FOR DEBUGGING ONLY.
        Validate the accuracy of the cross-mapping by comparing against
        direct transformation through physical coordinates.

        Args:
            stage_pos: Stage position as a numpy array of shape (2,)
            test_pixels: Optional test pixel coordinates. If None, uses a
            default set.

        Returns:
            dict: Dictionary containing validation metrics:
                - 'max_error_b2m': Maximum error for behavior-to-muscle transformation
                - 'max_error_m2b': Maximum error for muscle-to-behavior transformation
                - 'behavior_mapper_consistency': Round-trip error for behavior mapper
                - 'muscle_mapper_consistency': Round-trip error for muscle mapper
        """
        if test_pixels is None:
            test_pixels = np.array([[100, 200], [500, 600], [800, 400]])

        stage_expanded = np.tile(stage_pos, (test_pixels.shape[0], 1))

        # Test behavior-to-muscle transformation
        # Direct path: behavior pixels -> physical -> muscle pixels
        physical_coords = self.behavior_mapper.stage_and_pixel_to_physical(
            stage_expanded, test_pixels
        )
        muscle_direct = self.muscle_mapper.stage_and_physical_to_pixel(
            stage_expanded, physical_coords
        )

        # Cross-mapper path
        muscle_cross = self.map_pixel_coords_behavior2muscle(test_pixels, stage_pos)
        error_b2m = np.max(np.abs(muscle_direct - muscle_cross))

        # Test muscle-to-behavior transformation
        # Direct path: muscle pixels -> physical -> behavior pixels
        physical_coords_m = self.muscle_mapper.stage_and_pixel_to_physical(
            stage_expanded, test_pixels
        )
        behavior_direct = self.behavior_mapper.stage_and_physical_to_pixel(
            stage_expanded, physical_coords_m
        )

        # Cross-mapper path
        behavior_cross = self.map_pixel_coords_muscle2behavior(test_pixels, stage_pos)
        error_m2b = np.max(np.abs(behavior_direct - behavior_cross))

        # Test individual mapper consistency
        behavior_roundtrip = self.behavior_mapper.stage_and_physical_to_pixel(
            stage_expanded,
            self.behavior_mapper.stage_and_pixel_to_physical(
                stage_expanded, test_pixels
            ),
        )
        behavior_consistency = np.max(np.abs(test_pixels - behavior_roundtrip))

        muscle_roundtrip = self.muscle_mapper.stage_and_physical_to_pixel(
            stage_expanded,
            self.muscle_mapper.stage_and_pixel_to_physical(stage_expanded, test_pixels),
        )
        muscle_consistency = np.max(np.abs(test_pixels - muscle_roundtrip))

        return {
            "max_error_b2m": error_b2m,
            "max_error_m2b": error_m2b,
            "behavior_mapper_consistency": behavior_consistency,
            "muscle_mapper_consistency": muscle_consistency,
        }


class HomographyMapper:
    """
    Maps between behavior and muscle camera coordinates using homography transformation.

    This class provides methods to convert pixel coordinates between the two cameras
    using the homography matrix computed from ChArUco board calibration.

    Args:
        homography_parameters (dict | str | Path): Homography parameters
            as a dictionary, or a path to a YAML file containing the
            homography data.

    Example:
        >>> mapper = HomographyMapper("homography_result.yaml")
        >>> muscle_coords = mapper.behavior_to_muscle(behavior_coords)
        >>> behavior_coords = mapper.muscle_to_behavior(muscle_coords)
    """

    def __init__(self, homography_parameters: dict | str | Path):
        if isinstance(homography_parameters, (str, Path)):
            with open(homography_parameters, "r") as f:
                homography_parameters = yaml.safe_load(f)

        # Load homography matrices
        self.H_beh2muscle = np.array(
            homography_parameters["behavior_to_muscle"]["matrix"]
        )
        self.H_muscle2beh = np.array(
            homography_parameters["muscle_to_behavior"]["matrix"]
        )

        # Store metadata
        self.metadata = homography_parameters.get("metadata", {})

    def behavior_to_muscle(self, behavior_coords: np.ndarray) -> np.ndarray:
        """
        Transform coordinates from behavior camera to muscle camera.

        Args:
            behavior_coords: Array of shape (N, 2) or (2,) containing (x, y) coordinates
                in behavior camera pixel space

        Returns:
            Array of same shape containing (x, y) coordinates in muscle camera pixel space
        """
        # Handle single point
        single_point = False
        if behavior_coords.ndim == 1:
            behavior_coords = behavior_coords.reshape(1, -1)
            single_point = True

        # Convert to homogeneous coordinates
        ones = np.ones((behavior_coords.shape[0], 1))
        behavior_homogeneous = np.hstack([behavior_coords, ones])

        # Apply homography
        muscle_homogeneous = (self.H_beh2muscle @ behavior_homogeneous.T).T

        # Convert back to Cartesian coordinates
        muscle_coords = muscle_homogeneous[:, :2] / muscle_homogeneous[:, 2:3]

        if single_point:
            return muscle_coords.flatten()
        return muscle_coords

    def muscle_to_behavior(self, muscle_coords: np.ndarray) -> np.ndarray:
        """
        Transform coordinates from muscle camera to behavior camera.

        Args:
            muscle_coords: Array of shape (N, 2) or (2,) containing (x, y) coordinates
                in muscle camera pixel space

        Returns:
            Array of same shape containing (x, y) coordinates in behavior camera pixel space
        """
        # Handle single point
        single_point = False
        if muscle_coords.ndim == 1:
            muscle_coords = muscle_coords.reshape(1, -1)
            single_point = True

        # Convert to homogeneous coordinates
        ones = np.ones((muscle_coords.shape[0], 1))
        muscle_homogeneous = np.hstack([muscle_coords, ones])

        # Apply homography
        behavior_homogeneous = (self.H_muscle2beh @ muscle_homogeneous.T).T

        # Convert back to Cartesian coordinates
        behavior_coords = behavior_homogeneous[:, :2] / behavior_homogeneous[:, 2:3]

        if single_point:
            return behavior_coords.flatten()
        return behavior_coords

    def warp_image_behavior_to_muscle(
        self,
        behavior_image: np.ndarray,
        output_shape: tuple[int, int],
    ) -> np.ndarray:
        """
        Warp behavior camera image to muscle camera coordinate system.

        Args:
            behavior_image: Behavior camera image
            output_shape: Output image shape (height, width)

        Returns:
            Warped image in muscle camera coordinate system
        """
        return cv2.warpPerspective(
            behavior_image,
            self.H_beh2muscle,
            (output_shape[1], output_shape[0]),
            flags=cv2.INTER_LINEAR,
        )

    def warp_image_muscle_to_behavior(
        self,
        muscle_image: np.ndarray,
        output_shape: tuple[int, int],
    ) -> np.ndarray:
        """
        Warp muscle camera image to behavior camera coordinate system.

        Args:
            muscle_image: Muscle camera image
            output_shape: Output image shape (height, width)

        Returns:
            Warped image in behavior camera coordinate system
        """
        return cv2.warpPerspective(
            muscle_image,
            self.H_muscle2beh,
            (output_shape[1], output_shape[0]),
            flags=cv2.INTER_LINEAR,
        )
