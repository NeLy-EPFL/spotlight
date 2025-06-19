import numpy as np
import yaml
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
    as a dictionary or as a path to a YAML calibration file. The
    calibration file must contain version metadata and the required mapping
    coefficients.

    Args:
        calibration_parameters (dict | str | Path): Calibration parameters
            as a dictionary, or a path to a YAML file containing the
            calibration data.

    Raises:
        ValueError: If the calibration file version is incompatible or
            required metadata is missing.

    Example:
        >>> mapper = SpotlightPositionMapper("calibration.yaml")
        >>> pixel_coords = mapper.stage_and_physical_to_pixel(stage_pos, physical_pos)
        >>> physical_coords = mapper.stage_and_pixel_to_physical(stage_pos, pixel_coords)
    """

    _min_version_required = (1, 0, 0)  # Minimum version required in semver

    def __init__(self, calibration_parameters: dict | str | Path):
        if isinstance(calibration_parameters, (str, Path)):
            with open(calibration_parameters, "r") as f:
                calibration_parameters = yaml.safe_load(f)
            self._check_version_compatibility(calibration_parameters)

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

    def _check_version_compatibility(self, calibration_parameters: dict):
        """
        Check if the calibration file version is compatible with this mapper.

        Args:
            calibration_parameters: The loaded calibration parameters dictionary

        Raises:
            ValueError: If version is incompatible or format is incorrect
        """
        try:
            version_dict = calibration_parameters["metadata"]["file_format_version"]
            version_tuple = (
                version_dict["major"],
                version_dict["minor"],
                version_dict["patch"],
            )
            if version_tuple < self._min_version_required:
                raise ValueError(
                    f"Calibration file version {version_tuple} is less than the "
                    f"minimum required version {self._min_version_required}"
                )
        except KeyError:
            raise ValueError(
                "Calibration file does not contain 'file_format_version' metadata"
            )
