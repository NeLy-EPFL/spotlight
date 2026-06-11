#pragma once

#include <filesystem>
#include <thread>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/cli.h"
#include "recorder/common/data_types.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/muscle_camera.h"

void align_camera(const std::filesystem::path &profile_dir);
