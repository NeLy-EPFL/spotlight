#ifndef TRACKING_CONTROL_HPP
#define TRACKING_CONTROL_HPP

#include <iostream>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <spdlog/spdlog.h>

#include "peripherals/motionControl.hpp"
#include "global.hpp"
#include "constants.hpp"

// Function declarations
void motionControlRequestHandler();
MotionStagePosition getCurrentMotionStagePosition();
void setTargetMotionStagePosition(MotionStagePosition targetPosition);
void flyTrackingController(int tolerancePx, int gainPx);
void runCalibrationScanProcedure();
void stopMotionControlRequestHandler();

#endif // TRACKING_CONTROL_HPP