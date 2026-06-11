#include "recorder/peripherals/motion_control.h"

namespace {
zaber::motion::Units get_length_unit_zaber_enum(const std::string &unit_str) {
    if (unit_str == "mm") {
        return zaber::motion::Units::LENGTH_MILLIMETRES;
    } else {
        spdlog::critical("Length unit '{}' not supported", unit_str);
        throw std::runtime_error("Unsupported length unit.");
    }
}

zaber::motion::Units get_velocity_unit_zaber_enum(const std::string &unit_str) {
    if (unit_str == "mm/s") {
        return zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND;
    } else {
        spdlog::critical("Velocity unit '{}' not supported", unit_str);
        throw std::runtime_error("Unsupported velocity unit.");
    }
}
} // namespace

MotionControl::MotionControl(const RecorderConfig &recorder_config) {
    // Open the serial connection
    std::string motion_stage_device_manufacturer =
        recorder_config.get_parameter<std::string>(
            "motion_control", "motion_stage_device_manufacturer");
    std::string motion_stage_device_description =
        recorder_config.get_parameter<std::string>(
            "motion_control", "motion_stage_device_description");
    // get_serial_port_name(description, manufacturer): pass the arguments in
    // that order (matching the function signature and the Arduino call site).
    serial_port_name_ = get_serial_port_name(
        motion_stage_device_description, motion_stage_device_manufacturer);
    connection_ =
        zmASCII::Connection::openSerialPort("/dev/" + serial_port_name_);
    connection_.enableAlerts();

    // Detect devices
    std::vector<zmASCII::Device> device_list = connection_.detectDevices();

    if (device_list.size() != 1) {
        spdlog::critical(
            "Expected 1 Zaber device, but found {}. Note that multiple stages "
            "controlled by the same controller constitute a single device.",
            device_list.size());
        throw std::runtime_error("Unexpected number of Zaber devices");
    }

    zmASCII::Device device = device_list[0];
    int num_axis = device.getAxisCount();
    spdlog::info(
        "One Zaber device found. Device ID: {}; name: {}; axis count: {}; "
        "serial no.: {}",
        device.getDeviceId(),
        device.getName(),
        num_axis,
        device.getSerialNumber());

    // Configure axes
    if (num_axis != 2) {
        spdlog::critical("Expected 2 axes, found {}", num_axis);
        throw std::runtime_error("Unexpected number of axes");
    }

    int x_axis_serial_number = recorder_config.get_parameter<int>(
        "motion_control", "stage_serial_no_x_axis");
    int y_axis_serial_number = recorder_config.get_parameter<int>(
        "motion_control", "stage_serial_no_y_axis");
    serial_number_to_axis_lookup_ = {
        {x_axis_serial_number, x_axis}, {y_axis_serial_number, y_axis}};

    axis_ptr_lookup_[x_axis] = nullptr;
    axis_ptr_lookup_[y_axis] = nullptr;
    for (int i = 0; i < num_axis; i++) {
        zmASCII::Axis axis = device.getAxis(i + 1);
        unsigned int serial_number = axis.getPeripheralSerialNumber();
        spdlog::info(
            "Axis {}: peripheral ID: {}; name: {}, serial no.: {}",
            i + 1,
            axis.getPeripheralId(),
            axis.getPeripheralName(),
            serial_number);
        if (serial_number_to_axis_lookup_.find(serial_number) ==
            serial_number_to_axis_lookup_.end()) {
            spdlog::critical(
                "Motion stage serial number {} not mapped to any physically "
                "meaningful axis (ie. X or Y). Check config file.",
                serial_number);
            throw std::runtime_error("Unknown serial number");
        } else {
            axis_ptr_lookup_[serial_number_to_axis_lookup_.at(serial_number)] =
                std::make_unique<zmASCII::Axis>(std::move(axis));
        }
    }
    if (axis_ptr_lookup_[x_axis] == nullptr ||
        axis_ptr_lookup_[y_axis] == nullptr) {
        spdlog::critical(
            "Failed to configure all axes. X axis OK? {}; Y axis OK? {}",
            axis_ptr_lookup_[x_axis] != nullptr,
            axis_ptr_lookup_[y_axis] != nullptr);
        throw std::runtime_error("Failed to configure all axes");
    }
    spdlog::info("Motion stages assigned successfully");

    recorder_config_ = recorder_config;
    length_unit_enum_ =
        get_length_unit_zaber_enum(recorder_config.get_parameter<std::string>(
            "motion_control", "length_unit"));
    velocity_unit_enum_ =
        get_velocity_unit_zaber_enum(recorder_config.get_parameter<std::string>(
            "motion_control", "velocity_unit"));

    apply_motion_stage_settings(recorder_config);
}

void MotionControl::apply_motion_stage_settings(
    const RecorderConfig &recorder_config) {
    // The config keys carry their units in their names, so each setting is sent
    // with the matching Zaber unit (independent of the configured
    // length/velocity units used elsewhere).
    double acceleration = recorder_config.get_parameter<double>(
        "motion_control", "acceleration_mm_per_s_sq");
    double acceleration_ramp_time_ms = recorder_config.get_parameter<double>(
        "motion_control", "acceleration_ramp_time_ms");
    double max_speed = recorder_config.get_parameter<double>(
        "motion_control", "max_speed_mm_per_s");

    for (MotionAxis axis : {x_axis, y_axis}) {
        zmASCII::AxisSettings settings = axis_ptr_lookup_[axis]->getSettings();
        // Set "accel" alongside the accel-only and decel-only settings so that
        // acceleration and deceleration are both pinned explicitly.
        settings.set(
            "accel",
            acceleration,
            zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED);
        settings.set(
            "motion.accelonly",
            acceleration,
            zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED);
        settings.set(
            "motion.decelonly",
            acceleration,
            zaber::motion::Units::ACCELERATION_MILLIMETRES_PER_SECOND_SQUARED);
        settings.set(
            "motion.accel.ramptime",
            acceleration_ramp_time_ms,
            zaber::motion::Units::TIME_MILLISECONDS);
        settings.set(
            "maxspeed",
            max_speed,
            zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
    spdlog::info(
        "Applied motion stage settings to both axes: acceleration = {} mm/s^2 "
        "(accel, motion.accelonly, motion.decelonly); ramp time = {} ms "
        "(motion.accel.ramptime); max speed = {} mm/s (maxspeed)",
        acceleration,
        acceleration_ramp_time_ms,
        max_speed);
}

MotionControl::~MotionControl() {
    connection_.close();
    spdlog::info("Motion stages connection closed");
}

void MotionControl::move_absolute(
    MotionAxis axis, double position, bool wait, double velocity) {
    axis_ptr_lookup_[axis]->moveAbsolute(
        position, length_unit_enum_, wait, velocity, velocity_unit_enum_);
}

void MotionControl::move_relative(
    MotionAxis axis, double relative_position, bool wait, double velocity) {
    axis_ptr_lookup_[axis]->moveRelative(
        relative_position,
        length_unit_enum_,
        wait,
        velocity,
        velocity_unit_enum_);
}

void MotionControl::home(MotionAxis axis, bool wait) {
    axis_ptr_lookup_[axis]->home(wait);
}

double MotionControl::get_position(MotionAxis axis) {
    return axis_ptr_lookup_[axis]->getPosition(length_unit_enum_);
}

double MotionControl::get_min_position(MotionAxis axis) {
    return axis_ptr_lookup_[axis]->getSettings().get(
        "limit.min", length_unit_enum_);
}

double MotionControl::get_max_position(MotionAxis axis) {
    return axis_ptr_lookup_[axis]->getSettings().get(
        "limit.max", length_unit_enum_);
}

void MotionControl::wait_until_idle(MotionAxis axis) {
    axis_ptr_lookup_[axis]->waitUntilIdle();
}

bool MotionControl::check_if_idle(MotionAxis axis) {
    return !axis_ptr_lookup_[axis]->isBusy();
}