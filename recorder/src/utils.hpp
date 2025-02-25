#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <spdlog/spdlog.h>

#include "dataTypes.hpp"
#include "constants.hpp"

uint64_t getCurrentTimeMicroseconds();

cv::Mat makePseudoRGBImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);
std::string makeMetadataStringFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames);

std::string getSerialPortName(std::string deviceDescription,
                              std::string deviceManufacturer);

std::tuple<int, int> calculateMaxMotionStageRequestHandlingTime();

#endif // UTILS_HPP