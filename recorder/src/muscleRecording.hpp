#ifndef MUSCLE_RECORDING_HPP
#define MUSCLE_RECORDING_HPP

#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <fstream>

#include <spdlog/spdlog.h>

#include "peripherals/muscleCamera.hpp"
#include "recorderConfig.hpp"
#include "utils.hpp"

struct MuscleRecordingState
{
    std::shared_ptr<MuscleCamera> muscleCamera = nullptr;
    std::queue<FrameData> muscleImageQueue;
    std::mutex muscleImageQueueMutex;
    std::condition_variable muscleImageQueueCondVar;
    std::shared_ptr<LatestFrame> latestBehaviorFrameHolder;
};

class MuscleCameraROI
{
public:
    int x0;
    int x1;
    int y0;
    int y1;

    MuscleCameraROI(int x0, int x1, int y0, int y1);
    bool isWithinBound(int fullWidth, int fullHeight);
    int toFile(std::filesystem::path path);
};

// Function declarations
void muscleImageAcquierer(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    const RecorderConfig &recorderConfig,
    std::string profileDir,
    spdlog::level::level_enum logLevel,
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

MuscleCameraROI getMuscleCameraROI(std::filesystem::path roiFilePath);

#endif // MUSCLE_RECORDING_HPP