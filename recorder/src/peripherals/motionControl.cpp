#include "motionControl.hpp"

MotionControl::MotionControl()
{
    // Open the serial connection
    std::string serialPortName_ = getSerialPortName(
        MOTION_STAGE_DEVICE_DESCRIPTION, MOTION_STAGE_DEVICE_MANUFACTURER);
    connection_ = zmASCII::Connection::openSerialPort(
        "/dev/" + serialPortName_);
    connection_.enableAlerts();

    // Detect devices
    std::vector<zmASCII::Device> deviceList = connection_.detectDevices();

    if (deviceList.size() != 1)
    {
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
    if (numAxis != 2)
    {
        spdlog::critical("Expected 2 axes, found {}", numAxis);
        throw std::runtime_error("Unexpected number of axes");
    }
    axisPtrLookup_[X_AXIS] = nullptr;
    axisPtrLookup_[Y_AXIS] = nullptr;
    for (int i = 0; i < numAxis; i++)
    {
        zmASCII::Axis axis = device.getAxis(i + 1);
        unsigned int serialNumber = axis.getPeripheralSerialNumber();
        spdlog::info(
            "Axis {}: peripheral ID: {}; name: {}, serial no.: {}",
            i + 1,
            axis.getPeripheralId(),
            axis.getPeripheralName(),
            serialNumber);
        if (serialNumberToAxisLookup.find(serialNumber) ==
            serialNumberToAxisLookup.end())
        {
            spdlog::critical(
                "Motion stage serial number {} not mapped to any physically "
                "meaningful axis (ie. X or Y). Check `constants.hpp` and "
                "modify it as needed.",
                serialNumber);
            throw std::runtime_error("Unknown serial number");
        }
        else
        {
            axisPtrLookup_[serialNumberToAxisLookup.at(serialNumber)] =
                std::make_unique<zmASCII::Axis>(std::move(axis));
        }
    }
    if (axisPtrLookup_[X_AXIS] == nullptr || axisPtrLookup_[Y_AXIS] == nullptr)
    {
        spdlog::critical(
            "Failed to configure all axes. X axis OK? {}; Y axis OK? {}",
            axisPtrLookup_[X_AXIS] != nullptr,
            axisPtrLookup_[Y_AXIS] != nullptr);
        throw std::runtime_error("Failed to configure all axes");
    }
    spdlog::info("Motion stages assigned successfully");
}

MotionControl::~MotionControl()
{
    connection_.close();
    spdlog::info("Motion stages connection closed");
}

void MotionControl::moveAbsolute(
    MotionAxis axis, double position, bool wait, double velocity)
{
    axisPtrLookup_[axis]->moveAbsolute(
        position,
        MOTION_STAGE_LENGTH_UNIT,
        wait,
        velocity,
        MOTION_STAGE_VELOCITY_UNIT);
}

void MotionControl::moveRelative(
    MotionAxis axis, double relativePosition, bool wait, double velocity)
{
    axisPtrLookup_[axis]->moveRelative(
        relativePosition,
        MOTION_STAGE_LENGTH_UNIT,
        wait,
        velocity,
        MOTION_STAGE_VELOCITY_UNIT);
}

void MotionControl::home(MotionAxis axis, bool wait)
{
    axisPtrLookup_[axis]->home(wait);
}

double MotionControl::getPosition(MotionAxis axis)
{
    return axisPtrLookup_[axis]->getPosition(MOTION_STAGE_LENGTH_UNIT);
}

void MotionControl::waitUntilIdle(MotionAxis axis)
{
    axisPtrLookup_[axis]->waitUntilIdle();
}

bool MotionControl::checkIfIdle(MotionAxis axis)
{
    return !axisPtrLookup_[axis]->isBusy();
}