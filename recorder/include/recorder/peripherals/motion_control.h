#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>
#include <zaber/motion/ascii.h>

#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

namespace zmASCII = zaber::motion::ascii;

enum MotionAxis { x_axis, y_axis };

class MotionControl {
  public:
    MotionControl(const RecorderConfig &recorder_config);
    ~MotionControl();
    void
    move_absolute(MotionAxis axis, double position, bool wait, double velocity);
    void move_relative(
        MotionAxis axis, double relative_position, bool wait, double velocity);
    void home(MotionAxis axis, bool wait);
    double get_position(MotionAxis axis);
    double get_min_position(MotionAxis axis);
    double get_max_position(MotionAxis axis);
    void wait_until_idle(MotionAxis axis);
    bool check_if_idle(MotionAxis axis);

  private:
    // Push the explicit acceleration / ramp-time / max-speed settings from the
    // config onto both Zaber axes.
    void apply_motion_stage_settings(const RecorderConfig &recorder_config);

    RecorderConfig recorder_config_;
    std::unordered_map<unsigned int, MotionAxis> serial_number_to_axis_lookup_;
    std::string serial_port_name_;
    zmASCII::Connection connection_;
    std::unordered_map<MotionAxis, std::unique_ptr<zmASCII::Axis>>
        axis_ptr_lookup_;
    zaber::motion::Units length_unit_enum_;
    zaber::motion::Units velocity_unit_enum_;
};
