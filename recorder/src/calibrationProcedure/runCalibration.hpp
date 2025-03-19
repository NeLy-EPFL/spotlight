#ifndef RUN_CALIBRATION_HPP
#define RUN_CALIBRATION_HPP

#include <queue>
#include <atomic>
#include <filesystem>
#include <atomic>
#include <tuple>

#include <spdlog/spdlog.h>

#include "../cli.hpp"
#include "../recorderConfig.hpp"
#include "../utils.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/motionControl.hpp"

void runCalibrationScan(RecorderConfig &recorderConfig,
                        std::filesystem::path arucoSaveDir);

#endif // RUN_CALIBRATION_HPP