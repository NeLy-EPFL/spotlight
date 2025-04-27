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

#include "../peripherals/behaviorCamera.hpp"
#include "recorderConfig.hpp"
#include "utils.hpp"
#include "dataTypes.hpp"

struct BehaviorRecordingState
{
    std::shared_ptr<BehaviorCamera> behaviorCamera = nullptr;
    std::queue<GroupOfThreeFrames> behaviorImageQueue;
    std::mutex behaviorImageQueueMutex;
    std::condition_variable behaviorImageQueueCondVar;
    std::shared_ptr<LatestFrame> latestBehaviorFrameHolder;
};

BehaviorCameraROI getBehaviorBehaviorCameraROI(
    const RecorderConfig &recorderConfig);

// Function declarations
void behaviorImageAcquirer(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop);
void behaviorImageSaver(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState);
void stopBehaviorImageSaver(
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<ProgramState> programState);

#endif // BEHAVIOR_RECORDING_HPP