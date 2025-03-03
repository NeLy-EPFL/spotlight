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
#include "constants.hpp"
#include "utils.hpp"
#include "behaviorRecording.hpp"

// Hardware controller thread
void motionControlRequestHandler();

// Position logging thread
void motionStagePositionLogger();

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
void runCalibrationScanProcedureOneDirection(
    int currentlySetExposureTimeMicrosecs,
    CalibrationScanDirection scanDirection);
void runCalibrationScanProcedure(int currentlySetExposureTimeMicrosecs);

#endif // TRACKING_CONTROL_HPP