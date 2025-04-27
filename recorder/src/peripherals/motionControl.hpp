#ifndef MOTION_CONTROL_HPP
#define MOTION_CONTROL_HPP

#include <iostream>
#include <vector>
#include <unordered_map>

#include <zaber/motion/ascii.h>
#include <spdlog/spdlog.h>

#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"

namespace zmASCII = zaber::motion::ascii;

enum MotionAxis
{
    X_AXIS,
    Y_AXIS
};

class MotionControl
{
public:
    MotionControl(const RecorderConfig &recorderConfig);
    ~MotionControl();
    void moveAbsolute(
        MotionAxis axis,
        double position,
        bool wait,
        double velocity);
    void moveRelative(
        MotionAxis axis,
        double relativePosition,
        bool wait,
        double velocity);
    void home(MotionAxis axis, bool wait);
    double getPosition(MotionAxis axis);
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

#endif // MOTION_CONTROL_HPP
