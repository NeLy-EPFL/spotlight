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
// #include <memory>

#include <spdlog/spdlog.h>

#include "peripherals/behaviorCamera.hpp"
#include "global.hpp"
#include "recorderConfig.hpp"
#include "utils.hpp"

struct BehaviorRecordingState
{
    std::shared_ptr<BehaviorCamera> behaviorCamera = nullptr;
    std::queue<GroupOfThreeFrames> behaviorImageQueue;
    std::mutex behaviorImageQueueMutex;
    std::condition_variable behaviorImageQueueCondVar;
    std::queue<GroupOfThreeFrames> muscleImageQueue;
    std::mutex muscleImageQueueMutex;
    std::condition_variable muscleImageQueueCondVar;
};

// Function declarations
void behaviorImageAcquirer(const RecorderConfig &recorderConfig,
                           BehaviorRecordingState &behaviorRecordingState,
                           LatestFrame &latestBehaviorFrameHolder);
void behaviorImageSaver(const RecorderConfig &recorderConfig,
                        BehaviorRecordingState &behaviorRecordingState,
                        std::shared_ptr<SaveDirectory> saveDirectory);
void stopBehaviorImageSaver(BehaviorRecordingState &behaviorRecordingState);

#endif // BEHAVIOR_RECORDING_HPP