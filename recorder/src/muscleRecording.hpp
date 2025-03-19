#ifndef MUSCLE_RECORDING_HPP
#define MUSCLE_RECORDING_HPP

#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <fstream>

#include <spdlog/spdlog.h>

#include "recorderConfig.hpp"

// Function declarations
void muscleImageAcquierer();
void muscleImageSaver();

#endif // MUSCLE_RECORDING_HPP