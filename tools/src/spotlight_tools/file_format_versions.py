"""This module defines the required or specified versions for various
configuration, metadata, or parameter files. The constants should be kept
consistent with recorder/src/common/fileFormatVersions.hpp
"""

import logging


recorder_config_major = 1
recorder_config_minor = 0
recorder_config_patch = 0

calibration_result_major = 1
calibration_result_minor = 0
calibration_result_patch = 0

experiment_parameters_major = 1
experiment_parameters_minor = 0
experiment_parameters_patch = 0

muscle_camera_roi_major = 1
muscle_camera_roi_minor = 0
muscle_camera_roi_patch = 0


def check_version_compatibility(
    major: int, minor: int, specified_major: int, specified_minor: int
) -> bool:
    if major == specified_major and minor >= specified_minor:
        return True
    logging.error(
        f"Version mismatch: found {major}.{minor} but required "
        f"major version {specified_major} and minor version >={specified_minor}"
    )
    return False
