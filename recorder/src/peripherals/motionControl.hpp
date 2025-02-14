#ifndef MOTION_CONTROL_HPP
#define MOTION_CONTROL_HPP

#include <iostream>
#include <vector>
#include <unordered_map>

#include <zaber/motion/ascii.h>
#include <spdlog/spdlog.h>

#include "../constants.hpp"

namespace zmASCII = zaber::motion::ascii;

enum MotionAxis
{
    X_AXIS,
    Y_AXIS
};

const std::unordered_map<unsigned int, MotionAxis>
    serialNumberToAxisLookup = {
        {MOTION_STAGE_SERIAL_NUMBER_X_AXIS, X_AXIS},
        {MOTION_STAGE_SERIAL_NUMBER_Y_AXIS, Y_AXIS}};

class MotionControl
{
public:
    MotionControl(const std::string serialPort);
    ~MotionControl();
    void moveAbsolute(
        MotionAxis axis,
        double position,
        bool wait = true,
        double velocity = 0.0);
    void moveRelative(
        MotionAxis axis,
        double relativePosition,
        bool wait = true,
        double velocity = 0.0);
    void home(MotionAxis axis, bool wait = true);
    double getPosition(MotionAxis axis);
    void waitUntilIdle(MotionAxis axis);

private:
    std::string serialPort_;
    zmASCII::Connection connection_;
    std::unordered_map<MotionAxis, std::unique_ptr<zmASCII::Axis>> axisPtrLookup_;
};

#endif // MOTION_CONTROL_HPP
