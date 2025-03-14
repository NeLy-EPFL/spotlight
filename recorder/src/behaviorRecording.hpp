#ifndef BEHAVIOR_RECORDING_HPP
#define BEHAVIOR_RECORDING_HPP

#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <filesystem>
#include <set>

#include <spdlog/spdlog.h>

#include "peripherals/behaviorCamera.hpp"
#include "global.hpp"
#include "recorderConfig.hpp"

// Function declarations
void behaviorImageAcquierer(const RecorderConfig &recorderConfig);
void behaviorImageSaver();
void stopBehaviorImageSaver();

#endif // BEHAVIOR_RECORDING_HPP