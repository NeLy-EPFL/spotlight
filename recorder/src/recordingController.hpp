#ifndef RECORDING_CONTROLLER_HPP
#define RECORDING_CONTROLLER_HPP

#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

#include <spdlog/spdlog.h>

#include "peripherals/behaviorCamera.hpp"
#include "peripherals/motionControl.hpp"
#include "global.hpp"

void behaviorImageAcquierer();
void behaviorImageSaver(const std::string &directory);
void muscleImageAcquierer();
void muscleImageSaver(const std::string &directory);
void flyTrackingController(int tolerancePx, int gainPx);

#endif // RECORDING_CONTROLLER_HPP