#pragma once

// Declares run_charuco_homography_scan(profile_dir), which performs a grid
// scan over a small stage region to collect paired behavior+muscle images for
// homography fitting.
//
// Workflow
// --------
// 1. Both cameras are started and Arduino triggering is enabled (muscle imaging
//    on).
// 2. The stage visits each position of a serpentine grid defined by
//    [homography] scan_x_center_mm, scan_y_center_mm, scan_range_mm, and
//    scan_stride_mm in recorder_config.yaml.
// 3. At each position the stage is allowed to settle (200 ms sleep +
//    configurable settling frames), then 10 frame pairs are captured.
// 4. Images are saved to:
//      <profile_dir>/calibration/charuco_homography_scan/behavior_camera/
//          homography_scan_x{x:.2f}_y{y:.2f}_frame{i}.jpg
//      <profile_dir>/calibration/charuco_homography_scan/muscle_camera/
//          homography_scan_x{x:.2f}_y{y:.2f}_frame{i}.tif
//
// After the scan, run `fit_homography.py --profile-dir <profile_dir>` to fit
// the homography model.
//
// CLI:  run-charuco-homography-scan -p PROFILE_DIR [OPTIONS]
//       (see --help for details)

#include <filesystem>

#include <spdlog/spdlog.h>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/motion_control.h"
#include "recorder/peripherals/muscle_camera.h"

void run_charuco_homography_scan(const std::filesystem::path &profile_dir);
