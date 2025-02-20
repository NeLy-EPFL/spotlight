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

void behaviorImageAcquierer(
    std::shared_ptr<std::atomic<bool>> isRecording,
    std::shared_ptr<std::atomic<bool>> toQuit);
void behaviorImageSaver(
    const std::string &directory,
    std::shared_ptr<std::atomic<bool>> isRecording,
    std::shared_ptr<std::atomic<bool>> toQuit);
void muscleImageAcquierer(std::shared_ptr<std::atomic<bool>> toQuit);
void muscleImageSaver(const std::string &directory,
                      std::shared_ptr<std::atomic<bool>> toQuit);
void flyTrackingController(int tolerancePx,
                           int gainPx,
                           int speedMmPerSec,
                           std::shared_ptr<std::atomic<bool>> toQuit);

#endif // RECORDING_CONTROLLER_HPP