#ifndef RUN_CALIBRATION_HPP
#define RUN_CALIBRATION_HPP

#include <queue>
#include <atomic>
#include <filesystem>
#include <atomic>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <EGrabber.h>
#include <FormatConverter.h>
#include <spdlog/spdlog.h>
#include <QLabel>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <zaber/motion/ascii.h>

#include "../peripherals/behaviorCamera.hpp"
// #include "../peripherals/motionControl.hpp"
#include "../constants.hpp"
// #include "../utils.hpp"
// #include "../dataTypes.hpp"

namespace zmASCII = zaber::motion::ascii;

void runCalibrationScan();

#endif // RUN_CALIBRATION_HPP