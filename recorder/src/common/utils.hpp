#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>
#include <mutex>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <spdlog/spdlog.h>

#include "dataTypes.hpp"

namespace fs = std::filesystem;

uint64_t getCurrentTimeMicroseconds();

cv::Mat reorientBehaviorImage(cv::Mat image);
cv::Mat reorientMuscleImage(cv::Mat image);
cv::Mat makePseudoBGRImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);
std::string makeMetadataStringFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);

std::string getSerialPortName(std::string deviceDescription,
                              std::string deviceManufacturer);

int calculateBehaviorCameraPreviewWidth(
    int behaviorCameraPreviewHeight,
    int motionStageXRange,
    int motionStageYRange);

fs::path prepareOutputFolder(const fs::path &directory, bool clearFolder);

size_t getMyThreadIdHash();

std::string expandPath(const std::string& path);

class SaveDirectory
{
public:
    SaveDirectory(std::string directory);
    void setDirectory(std::string directory);
    std::filesystem::path getDirectory();
    void initialize();

private:
    std::mutex mutex_;
    fs::path directory_;
};

class LatestFrame
{
public:
    LatestFrame();
    FrameData getLatestFrameData();
    void setLatestFrameData(FrameData frameData);

private:
    FrameData latestFrameData_;
    std::mutex latestFrameMutex_;
};

#endif // UTILS_HPP