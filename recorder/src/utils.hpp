#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>

#include <QSerialPortInfo>
#include <QDebug>
#include <spdlog/spdlog.h>

#include "dataTypes.hpp"

uint64_t getCurrentTimeMicroseconds();
std::string getSerialPortName(
    std::string deviceDescription = "Nano ESP32",
    std::string deviceManufacturer = "Arduino");

cv::Mat makePseudoRGBImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);
std::string makeMetadataStringFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);

class CameraAcquisitionConfig
{
public:
    CameraAcquisitionConfig(CameraAcquisitionMode mode,
                            int fps,
                            int exposureTimeMicroseconds);
    CameraAcquisitionConfig(std::string commandString);
    std::string toCommandString() const;

    CameraAcquisitionMode mode;
    int fps;
    int exposureTimeMicroseconds;
};

#endif // UTILS_HPP
