#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>

#include <spdlog/spdlog.h>

#include "dataTypes.hpp"

uint64_t getCurrentTimeMicroseconds();

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
