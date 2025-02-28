#ifndef BEHAVIOR_RECORDING_HPP
#define BEHAVIOR_RECORDING_HPP

#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <filesystem>

#include <spdlog/spdlog.h>

#include "peripherals/behaviorCamera.hpp"
#include "global.hpp"
#include "constants.hpp"

// Function declarations
void behaviorImageAcquierer();
void behaviorImageSaver();
void stopBehaviorImageSaver();

#endif // BEHAVIOR_RECORDING_HPP