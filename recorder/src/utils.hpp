#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <spdlog/spdlog.h>

#include "dataTypes.hpp"

namespace fs = std::filesystem;

uint64_t getCurrentTimeMicroseconds();

cv::Mat correctImageRotationAndFlip(cv::Mat image);
cv::Mat makePseudoRGBImageFromThreeFrames(
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

#endif // UTILS_HPP