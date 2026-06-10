#pragma once

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <tuple>

#include <spdlog/spdlog.h>

#include "recorder/peripherals/muscle_camera.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

struct MuscleRecordingState {
    std::shared_ptr<MuscleCamera> muscleCamera = nullptr;
    std::queue<FrameData> muscleImageQueue;
    std::mutex muscleImageQueueMutex;
    std::condition_variable muscleImageQueueCondVar;
    std::shared_ptr<LatestFrame> latestFrameHolder;
};

class MuscleCameraROI {
  public:
    int x0;
    int x1;
    int y0;
    int y1;
    int xOffset;
    int yOffset;
    int imageWidth;
    int imageHeight;

    MuscleCameraROI(int x0, int x1, int y0, int y1);
    bool isWithinBound(int fullWidth, int fullHeight) const;
    int toFile(const std::filesystem::path &path) const;
    std::tuple<int, int> getCenterXY() const;
};

// Function declarations
void muscleImageAcquirer(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    const RecorderConfig &recorderConfig,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop);

void muscleImageSaver(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState,
    int tiffCompressionMethod = 5); // see below
// TIFF compression methods:
//   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
//   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
//   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
//   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
//   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html

void stopMuscleImageSaver(
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState);

MuscleCameraROI getMuscleCameraROI(const std::filesystem::path &roiFilePath);
