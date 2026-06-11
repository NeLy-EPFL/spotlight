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

#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/behavior_camera.h"

struct BehaviorRecordingState {
    std::shared_ptr<BehaviorCamera> behavior_camera = nullptr;
    std::queue<GroupOfThreeFrames> behavior_image_queue;
    std::mutex behavior_image_queue_mutex;
    std::condition_variable behavior_image_queue_cond_var;
    std::shared_ptr<LatestFrame> latest_frame_holder;
};

BehaviorCameraROI
get_behavior_behavior_camera_roi(const RecorderConfig &recorder_config);

// Function declarations
void behavior_image_acquirer(
    const RecorderConfig &recorder_config,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    const std::shared_ptr<ProgramState>& program_state,
    const std::shared_ptr<ProgrammedStop>& programmed_recording_stop);
void behavior_image_saver(
    const RecorderConfig &recorder_config,
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state,
    const std::shared_ptr<SaveDirectory>& save_directory,
    const std::shared_ptr<ProgramState>& program_state);
void stop_behavior_image_saver(
    const std::shared_ptr<BehaviorRecordingState>& behavior_recording_state,
    const std::shared_ptr<ProgramState>& program_state);
