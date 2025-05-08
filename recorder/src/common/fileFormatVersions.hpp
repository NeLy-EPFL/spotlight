/**
 * This file defines the required file format versions for the
 * recorder_config.yaml, calibration_result.yaml, experiment_parameters.yaml,
 * and muscle_camera_roi.yaml files.
 *
 * Semantic versioning (www.semver.org) is used to define the file format
 * versions. Therefore, the major version must be an exact match, while the
 * minor version can be anything that's greater than or equal to the specified
 * minor version. Patch versions are ignored in the verification procedure.
 */

#ifndef FILE_FORMAT_VERSIONS
#define FILE_FORMAT_VERSIONS

#define RECORDER_CONFIG_MAJOR 1
#define RECORDER_CONFIG_MINOR 0
#define RECORDER_CONFIG_PATCH 0

#define CALIBRATION_RESULT_MAJOR 1
#define CALIBRATION_RESULT_MINOR 0
#define CALIBRATION_RESULT_PATCH 0

#define EXPERIMENT_PARAMETERS_MAJOR 1
#define EXPERIMENT_PARAMETERS_MINOR 0
#define EXPERIMENT_PARAMETERS_PATCH 0

#define MUSCLE_CAMERA_ROI_MAJOR 1
#define MUSCLE_CAMERA_ROI_MINOR 0
#define MUSCLE_CAMERA_ROI_PATCH 0

#include <spdlog/spdlog.h>

bool checkVersionCompatibility(
    int major, int minor, int specifiedMajor, int specifiedMinor);

#endif // FILE_FORMAT_VERSIONS