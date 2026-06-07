#include "recorder/peripherals/motion_control.h"

namespace {
zaber::motion::Units getLengthUnitZaberEnum(const std::string &unitStr) {
    if (unitStr == "mm") {
        return zaber::motion::Units::LENGTH_MILLIMETRES;
    } else {
        spdlog::critical("Length unit '{}' not supported", unitStr);
        throw std::runtime_error("Unsupported length unit.");
    }
}

zaber::motion::Units getVelocityUnitZaberEnum(const std::string &unitStr) {
    if (unitStr == "mm/s") {
        return zaber::motion::Units::VELOCITY_MILLIMETRES_PER_SECOND;
    } else {
        spdlog::critical("Velocity unit '{}' not supported", unitStr);
        throw std::runtime_error("Unsupported velocity unit.");
    }
}
} // namespace

MotionControl::MotionControl(const RecorderConfig &recorderConfig) {
    // Open the serial connection
    std::string motionStageDeviceManufacturer =
        recorderConfig.getParameter<std::string>(
            "motion_control", "motion_stage_device_manufacturer");
    std::string motionStageDeviceDescription =
        recorderConfig.getParameter<std::string>(
            "motion_control", "motion_stage_device_description");
    // getSerialPortName(description, manufacturer): pass the arguments in that
    // order (matching the function signature and the Arduino call site).
    std::string serialPortName_ = getSerialPortName(
        motionStageDeviceDescription, motionStageDeviceManufacturer);
    connection_ =
        zmASCII::Connection::openSerialPort("/dev/" + serialPortName_);
    connection_.enableAlerts();

    // Detect devices
    std::vector<zmASCII::Device> deviceList = connection_.detectDevices();

    if (deviceList.size() != 1) {
        spdlog::critical(
            "Expected 1 Zaber device, but found {}. Note that multiple stages "
            "controlled by the same controller constitute a single device.",
            deviceList.size());
        throw std::runtime_error("Unexpected number of Zaber devices");
    }

    zmASCII::Device device = deviceList[0];
    int numAxis = device.getAxisCount();
    spdlog::info(
        "One Zaber device found. Device ID: {}; name: {}; axis count: {}; "
        "serial no.: {}",
        device.getDeviceId(),
        device.getName(),
        numAxis,
        device.getSerialNumber());

    // Configure axes
    if (numAxis != 2) {
        spdlog::critical("Expected 2 axes, found {}", numAxis);
        throw std::runtime_error("Unexpected number of axes");
    }

    int xAxisSerialNumber = recorderConfig.getParameter<int>(
        "motion_control", "stage_serial_no_x_axis");
    int yAxisSerialNumber = recorderConfig.getParameter<int>(
        "motion_control", "stage_serial_no_y_axis");
    serialNumberToAxisLookup_ = {
        {xAxisSerialNumber, X_AXIS}, {yAxisSerialNumber, Y_AXIS}};

    axisPtrLookup_[X_AXIS] = nullptr;
    axisPtrLookup_[Y_AXIS] = nullptr;
    for (int i = 0; i < numAxis; i++) {
        zmASCII::Axis axis = device.getAxis(i + 1);
        unsigned int serialNumber = axis.getPeripheralSerialNumber();
        spdlog::info(
            "Axis {}: peripheral ID: {}; name: {}, serial no.: {}",
            i + 1,
            axis.getPeripheralId(),
            axis.getPeripheralName(),
            serialNumber);
        if (serialNumberToAxisLookup_.find(serialNumber) ==
            serialNumberToAxisLookup_.end()) {
            spdlog::critical(
                "Motion stage serial number {} not mapped to any physically "
                "meaningful axis (ie. X or Y). Check config file.",
                serialNumber);
            throw std::runtime_error("Unknown serial number");
        } else {
            axisPtrLookup_[serialNumberToAxisLookup_.at(serialNumber)] =
                std::make_unique<zmASCII::Axis>(std::move(axis));
        }
    }
    if (axisPtrLookup_[X_AXIS] == nullptr ||
        axisPtrLookup_[Y_AXIS] == nullptr) {
        spdlog::critical(
            "Failed to configure all axes. X axis OK? {}; Y axis OK? {}",
            axisPtrLookup_[X_AXIS] != nullptr,
            axisPtrLookup_[Y_AXIS] != nullptr);
        throw std::runtime_error("Failed to configure all axes");
    }
    spdlog::info("Motion stages assigned successfully");

    recorderConfig_ = recorderConfig;
    lengthUnitEnum_ =
        getLengthUnitZaberEnum(recorderConfig.getParameter<std::string>(
            "motion_control", "length_unit"));
    velocityUnitEnum_ =
        getVelocityUnitZaberEnum(recorderConfig.getParameter<std::string>(
            "motion_control", "velocity_unit"));
}

MotionControl::~MotionControl() {
    connection_.close();
    spdlog::info("Motion stages connection closed");
}

void MotionControl::moveAbsolute(
    MotionAxis axis, double position, bool wait, double velocity) {
    axisPtrLookup_[axis]->moveAbsolute(
        position, lengthUnitEnum_, wait, velocity, velocityUnitEnum_);
}

void MotionControl::moveRelative(
    MotionAxis axis, double relativePosition, bool wait, double velocity) {
    axisPtrLookup_[axis]->moveRelative(
        relativePosition, lengthUnitEnum_, wait, velocity, velocityUnitEnum_);
}

void MotionControl::home(MotionAxis axis, bool wait) {
    axisPtrLookup_[axis]->home(wait);
}

double MotionControl::getPosition(MotionAxis axis) {
    return axisPtrLookup_[axis]->getPosition(lengthUnitEnum_);
}

double MotionControl::getMinPosition(MotionAxis axis) {
    return axisPtrLookup_[axis]->getSettings().get(
        "limit.min", lengthUnitEnum_);
}

double MotionControl::getMaxPosition(MotionAxis axis) {
    return axisPtrLookup_[axis]->getSettings().get(
        "limit.max", lengthUnitEnum_);
}

void MotionControl::waitUntilIdle(MotionAxis axis) {
    axisPtrLookup_[axis]->waitUntilIdle();
}

bool MotionControl::checkIfIdle(MotionAxis axis) {
    return !axisPtrLookup_[axis]->isBusy();
}