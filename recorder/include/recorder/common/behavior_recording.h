#pragma once

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <set>

#include <spdlog/spdlog.h>

#include "recorder/peripherals/behavior_camera.h"
#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

struct BehaviorRecordingState {
    std::shared_ptr<BehaviorCamera> behaviorCamera = nullptr;
    std::queue<GroupOfThreeFrames> behaviorImageQueue;
    std::mutex behaviorImageQueueMutex;
    std::condition_variable behaviorImageQueueCondVar;
    std::shared_ptr<LatestFrame> latestFrameHolder;
};

BehaviorCameraROI
getBehaviorBehaviorCameraROI(const RecorderConfig &recorderConfig);

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
