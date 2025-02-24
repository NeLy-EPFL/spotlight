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

#endif // UTILS_HPP