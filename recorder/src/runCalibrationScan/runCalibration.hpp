#ifndef RUN_CALIBRATION_HPP
#define RUN_CALIBRATION_HPP

#include <queue>
#include <atomic>
#include <filesystem>
#include <atomic>
#include <tuple>

#include <spdlog/spdlog.h>

#include "../common/cli.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "../common/behaviorRecording.hpp"
#include "../common/muscleRecording.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/motionControl.hpp"
#include "../peripherals/muscleCamera.hpp"
#include "../peripherals/arduinoCommunication.hpp"

void runCalibrationScan(std::filesystem::path profileDir,
                        std::filesystem::path arucoSaveDir);

#endif // RUN_CALIBRATION_HPP