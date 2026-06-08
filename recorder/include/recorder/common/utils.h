#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/data_types.h"

namespace fs = std::filesystem;

uint64_t getCurrentTimeMicroseconds();

void reorientBehaviorImage(const cv::Mat &sourceImage, cv::Mat &targetImage);
void reorientMuscleImage(const cv::Mat &sourceImage, cv::Mat &targetImage);
cv::Mat
makePseudoBGRImageFromThreeFrames(const GroupOfThreeFrames &groupOfThreeFrames);
std::string
makeMetadataStringFromThreeFrames(const GroupOfThreeFrames &groupOfThreeFrames);

std::string getSerialPortName(
    const std::string &deviceDescription,
    const std::string &deviceManufacturer);

int calculateBehaviorCameraPreviewWidth(
    int behaviorCameraPreviewHeight,
    int motionStageXRange,
    int motionStageYRange);

fs::path prepareOutputFolder(const fs::path &directory, bool clearFolder);

size_t getMyThreadIdHash();

std::string expandPath(const std::string &path);

void convert16BitTo8Bit(
    const cv::Mat &sourceImage, cv::Mat &targetImage, int scale, int offset);

// The muscle_nominal_exposure_us and muscle_buffer_time_us values are the
// derived continuous-mode (auto-sequence) timing (see MuscleTriggerTiming): the
// nominal per-line exposure programmed into the camera, and the slack in the
// common-time window beyond the light-on time. They are only written when
// muscle_imaging_enabled is true.
void writeExperimentParameters(
    const std::filesystem::path &outputPath,
    int behavior_fps,
    bool muscle_imaging_enabled,
    int muscle_sync_ratio,
    int behavior_exposure_time_us,
    int muscle_light_on_time_us,
    int muscle_nominal_exposure_us,
    int muscle_buffer_time_us,
    const std::string &experiment_protocol);

class SaveDirectory {
  public:
    SaveDirectory(const std::string &directory);
    void setDirectory(const std::string &directory);
    std::filesystem::path getDirectory();
    void initialize();

  private:
    std::mutex mutex_;
    fs::path directory_;
};

class LatestFrame {
  public:
    LatestFrame();
    FrameData getLatestFrameData();
    void setLatestFrameData(const FrameData &frameData);

  private:
    FrameData latestFrameData_;
    std::mutex latestFrameMutex_;
};
