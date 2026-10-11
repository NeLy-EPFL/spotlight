#pragma once

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <tuple>

#include <spdlog/spdlog.h>

#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/muscle_camera.h"

struct MuscleRecordingState {
    // Published by the acquirer thread once the camera is constructed and read
    // by several other threads (GUI, init wait loop, shutdown). atomic so that
    // publication and reads are not a data race; load()/store() it.
    std::atomic<std::shared_ptr<MuscleCamera>> muscle_camera;
    std::queue<FramePair> muscle_image_queue;
    std::mutex muscle_image_queue_mutex;
    std::condition_variable muscle_image_queue_cond_var;
    // Latest frame of each muscle camera, for live previews.
    std::shared_ptr<LatestFrame> latest_calcium_frame_holder =
        std::make_shared<LatestFrame>();
    std::shared_ptr<LatestFrame> latest_fiducial_frame_holder =
        std::make_shared<LatestFrame>();
};

// Function declarations
void muscle_image_acquirer(
    const MuscleCameraROIs &rois,
    const RecorderConfig &recorder_config,
    const std::string &profile_dir,
    spdlog::level::level_enum log_level,
    const std::shared_ptr<MuscleRecordingState> &muscle_recording_state,
    const std::shared_ptr<ProgramState> &program_state,
    const std::shared_ptr<ProgrammedStop> &programmed_recording_stop);

void muscle_image_saver(
    const RecorderConfig &recorder_config,
    std::shared_ptr<MuscleRecordingState> muscle_recording_state,
    const std::shared_ptr<SaveDirectory> &save_directory,
    const std::shared_ptr<ProgramState> &program_state,
    int tiff_compression_method = 5); // see below
// TIFF compression methods:
//   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
//   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
//   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
//   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
//   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html

void stop_muscle_image_saver(
    const std::shared_ptr<MuscleRecordingState> &muscle_recording_state,
    const std::shared_ptr<ProgramState> &program_state);
