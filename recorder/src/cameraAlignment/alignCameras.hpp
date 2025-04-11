#ifndef ALIGN_CAMERA_HPP
#define ALIGN_CAMERA_HPP

#include <filesystem>
#include <thread>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/muscleCamera.hpp"
#include "../behaviorRecording.hpp"
#include "../muscleRecording.hpp"
#include "../utils.hpp"
#include "../recorderConfig.hpp"
#include "../dataTypes.hpp"
#include "../cli.hpp"

void alignCamera(RecorderConfig &RecorderConfig);

#endif // ALIGN_CAMERA_HPP