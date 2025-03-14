#ifndef TRACKING_CONTROL_HPP
#define TRACKING_CONTROL_HPP

#include <iostream>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <future>
#include <thread>
#include <limits>
#include <filesystem>
#include <set>

#include <spdlog/spdlog.h>

#include "peripherals/motionControl.hpp"
#include "peripherals/triggering.hpp"
#include "global.hpp"
#include "utils.hpp"
#include "behaviorRecording.hpp"
#include "calibration.hpp"
#include "recorderConfig.hpp"

// Hardware controller thread
void motionControlRequestHandler(const RecorderConfig &recorderConfig);

// Tracking thread
void trackingController(const RecorderConfig &recorderConfig);
cv::Mat blackoutOutside(cv::Mat image,
                        MotionStagePosition stagePos,
                        const RecorderConfig &recorderConfig);

// Position logging thread
void motionStagePositionLogger(const RecorderConfig &recorderConfig);

// Global API functions
// Aside from getCurrentMotionStagePosition(), they are all async.
MotionStagePosition getCurrentMotionStagePosition();
void setTargetMotionStagePosition(
    MotionStagePosition targetPosition,
    float velocity = MOTION_STAGE_DEFAULT_VELOCITY_MM_PER_SEC);
void waitUntilMotionStageIdleSync();
void waitUntilMotionStageIdleAsync();
bool checkIfMotionStageIdle();
void startHomingMotionStage();
void stopMotionControlRequestHandler();

// High-level helper functions
std::tuple<bool, double, double> calculateFlyPositionAbsoluteMm(
    cv::Mat behaviorImage,
    MotionStagePosition stagePosition,
    const RecorderConfig &recorderConfig);

#endif // TRACKING_CONTROL_HPP