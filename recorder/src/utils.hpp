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

#endif // UTILS_HPP
