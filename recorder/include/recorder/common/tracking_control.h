#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <set>
#include <thread>
#include <tuple>

#include <spdlog/spdlog.h>

#include "recorder/peripherals/motion_control.h"
#include "recorder/common/behavior_recording.h"
#include "recorder/common/calibration.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

class ActiveAreaMask {
  public:
    cv::Mat fullArenaMask;
    double resolutionMmPerPixel;
    double arenaWidthMm;
    double arenaHeightMm;
    LinearMapper2x2to2 &stageAndPixelToPhysical;

    ActiveAreaMask(
        const std::string &arenaSpecDir,
        double boundaryMarginMm,
        LinearMapper2x2to2 &stageAndPixelToPhysical);
    cv::Mat warpToCurrentView(
        const cv::Mat &currentImage, MotionStagePosition stagePos) const;

  private:
    cv::Mat transformMatrixAtZeroStagePos_;
};

struct TrackingControlState {
    std::atomic<bool> motionControlHandlerReady = false;
    MotionStagePosition latestMotionStagePosition;
    std::mutex latestMotionStagePositionMutex;
    std::atomic<bool> trackingOn = true;
    std::atomic<bool> shouldOverrideTracking = false;
    std::atomic<double> overridingPosX; // in mm
    std::atomic<double> overridingPosY; // in mm
};

// Hardware controller thread
void motionControlRequestHandler(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<TrackingControlState> trackingControlState,
    std::shared_ptr<ProgramState> programState);

// Tracking thread
void trackingController(
    const RecorderConfig &recorderConfig,
    ActiveAreaMask &activeAreaMask,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<TrackingControlState> trackingControlState,
    const CalibrationParams &behaviorCamCalibrationParams,
    std::shared_ptr<ProgramState> programState);

// Position logging thread
void motionStagePositionLogger(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<TrackingControlState> trackingControlState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState);

// Global API functions
// Aside from getCurrentMotionStagePosition(), they are all async.
MotionStagePosition getCurrentMotionStagePosition();
void setTargetMotionStagePosition(
    MotionStagePosition targetPosition, float velocity);
void setMotionStageLimits(
    double xMinMm, double xMaxMm, double yMinMm, double yMaxMm);
void waitUntilMotionStageIdleSync();
void waitUntilMotionStageIdleAsync();
bool checkIfMotionStageIdle();
void startHomingMotionStage();
void stopMotionControlRequestHandler(
    std::shared_ptr<ProgramState> programState);

// High-level helper functions
std::tuple<bool, double, double> calculateFlyPositionAbsoluteMm(
    const cv::Mat &behaviorImage,
    MotionStagePosition stagePosition,
    const cv::Mat &activeAreaMaskCurrView,
    const CalibrationParams &behaviorCamCalibrationParams,
    const RecorderConfig &recorderConfig);
