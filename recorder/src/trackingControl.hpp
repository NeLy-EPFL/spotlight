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
#include <tuple>

#include <spdlog/spdlog.h>

#include "peripherals/motionControl.hpp"
#include "peripherals/triggering.hpp"
#include "global.hpp"
#include "utils.hpp"
#include "behaviorRecording.hpp"
#include "calibration.hpp"
#include "recorderConfig.hpp"

struct TrackingControlState
{
    std::atomic<bool> motionControlHandlerReady = false;
    MotionStagePosition latestMotionStagePosition;
    std::mutex latestMotionStagePositionMutex;
    std::atomic<bool> shouldOverrideTracking = false;
    std::atomic<double> overridingPosX; // in mm
    std::atomic<double> overridingPosY; // in mm
};

// Hardware controller thread
void motionControlRequestHandler(const RecorderConfig &recorderConfig,
                                 TrackingControlState &trackingControlState);

// Tracking thread
void trackingController(const RecorderConfig &recorderConfig,
                        BehaviorRecordingState &behaviorRecordingState,
                        TrackingControlState &trackingControlState);

// Position logging thread
void motionStagePositionLogger(const RecorderConfig &recorderConfig,
                               TrackingControlState &trackingControlState);

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

cv::Mat blackoutOutside(cv::Mat image,
                        MotionStagePosition stagePos,
                        const RecorderConfig &recorderConfig);

#endif // TRACKING_CONTROL_HPP