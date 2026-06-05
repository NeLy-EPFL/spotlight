#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>
#include <zaber/motion/ascii.h>

#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

namespace zmASCII = zaber::motion::ascii;

enum MotionAxis { X_AXIS, Y_AXIS };

class MotionControl {
  public:
    MotionControl(const RecorderConfig &recorderConfig);
    ~MotionControl();
    void
    moveAbsolute(MotionAxis axis, double position, bool wait, double velocity);
    void moveRelative(
        MotionAxis axis, double relativePosition, bool wait, double velocity);
    void home(MotionAxis axis, bool wait);
    double getPosition(MotionAxis axis);
    double getMinPosition(MotionAxis axis);
    double getMaxPosition(MotionAxis axis);
    void waitUntilIdle(MotionAxis axis);
    bool checkIfIdle(MotionAxis axis);

  private:
    RecorderConfig recorderConfig_;
    std::unordered_map<unsigned int, MotionAxis> serialNumberToAxisLookup_;
    std::string serialPortName_;
    zmASCII::Connection connection_;
    std::unordered_map<MotionAxis, std::unique_ptr<zmASCII::Axis>>
        axisPtrLookup_;
    zaber::motion::Units lengthUnitEnum_;
    zaber::motion::Units velocityUnitEnum_;
};

