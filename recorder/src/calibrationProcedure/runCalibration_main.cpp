#include "runCalibration.hpp"

namespace mine
{
    const int calibrationStreamFrameRate = 25;

    fs::path prepareOutputFolder(const fs::path &directory, bool clearFolder)
    {
        fs::path processedDir;

        // Expand ~ to home directory
        if (!directory.empty() && directory.string().front() == '~')
        {
            const char *homeDir = getenv("HOME");
            if (homeDir)
            {
                processedDir = fs::path(homeDir) / directory.string().substr(2);
                spdlog::info("Expanded ~ in directory path '{}' to '{}'",
                             directory.string(),
                             processedDir.string());
            }
            else
            {
                spdlog::error(
                    "Failed to expand ~ in directory path '{}' because $HOME is "
                    "not defined. Set the $HOME environment variable or use "
                    "absolute path.",
                    directory.string());
            }
        }
        else
        {
            processedDir = directory;
        }
        fs::path absoluteDir = fs::absolute(processedDir);

        try
        {
            fs::create_directories(absoluteDir);
            spdlog::info("Created directory '{}' (if it didn't already exist)",
                         absoluteDir.string());

            if (clearFolder)
            {
                for (const auto &entry : fs::directory_iterator(absoluteDir))
                {
                    fs::remove_all(entry);
                }
                spdlog::info("Cleared content of directory '{}'",
                             absoluteDir.string());
            }
        }
        catch (const fs::filesystem_error &e)
        {
            spdlog::error(
                "Failed to create directory '{}' or clear its content: {}",
                absoluteDir.string(), e.what());
            throw;
        }

        return absoluteDir;
    }

    enum MyMotionAxis
    {
        X_AXIS,
        Y_AXIS
    };

    const std::unordered_map<unsigned int, MyMotionAxis>
        serialNumberToAxisLookup = {
            {MOTION_STAGE_SERIAL_NUMBER_X_AXIS, X_AXIS},
            {MOTION_STAGE_SERIAL_NUMBER_Y_AXIS, Y_AXIS}};

    zmASCII::Connection connection_;

    std::unordered_map<MyMotionAxis, std::unique_ptr<zmASCII::Axis>>
        axisPtrLookup_;

    std::string getSerialPortName(std::string deviceDescription,
                                  std::string deviceManufacturer)
    {
        std::vector<SerialPortInfo> allSerialPortInfo;

        foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
        {
            std::string portName = port.portName().toStdString();
            std::string description = port.description().toStdString();
            std::string manufacturer = port.manufacturer().toStdString();
            if (description == deviceDescription &&
                manufacturer == deviceManufacturer)
            {
                spdlog::info(
                    "Serial port found. "
                    "Port name: '{}', description: '{}', manufacturer: '{}'",
                    portName, description, manufacturer);
                return portName;
            }
            allSerialPortInfo.push_back({portName, description, manufacturer});
        }

        spdlog::critical(
            "Serial port not found. "
            "I'm looking for manufacturer '{}', description '{}'. "
            "Available ports are:",
            deviceManufacturer, deviceDescription);
        for (SerialPortInfo serialPortInfo : allSerialPortInfo)
        {
            spdlog::critical(
                "* Port name: '{}', description: '{}', manufacturer: '{}'",
                serialPortInfo.portName,
                serialPortInfo.description,
                serialPortInfo.manufacturer);
        }
        throw std::runtime_error("Serial port not found.");
        return "";
    }

    void setupMotionControl()
    {
        // Open the serial connection
        std::string serialPortName_ = mine::getSerialPortName(
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
}

void runCalibrationScan()
{
    std::filesystem::path saveDir =
        mine::prepareOutputFolder(SPOTLIGHT_ARUCO_SCAN_DIR, true);

    unsigned int imageWidth = roundToMultiplesOf64(
        BEHAVIOR_CAMERA_ROI_WIDTH);
    unsigned int imageHeight = roundToMultiplesOf64(
        BEHAVIOR_CAMERA_ROI_HEIGHT);
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    std::atomic<bool> cameraReadyFlag(false);
    BehaviorCamera behaviorCamera(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        BEHAVIOR_CAMERA_FRAME_GRABBER_TRIGGER_LINE,
        cameraReadyFlag);

    spdlog::info("Behavior camera configured");

    mine::setupMotionControl();

    std::queue<std::tuple<double, double>> calibrationPositions;
    int numColumns =
        (MOTION_STAGE_X_MAX_PHYSICAL_MM - MOTION_STAGE_X_MIN_PHYSICAL_MM) /
            CALIBRATION_SCAN_STRIDE_MM +
        1;
    int numRows =
        (MOTION_STAGE_Y_MAX_PHYSICAL_MM - MOTION_STAGE_Y_MIN_PHYSICAL_MM) /
            CALIBRATION_SCAN_STRIDE_MM +
        1;
    for (int iRow = 0; iRow < numRows; iRow++)
    {
        double yPosMm = MOTION_STAGE_X_MIN_PHYSICAL_MM +
                        iRow * CALIBRATION_SCAN_STRIDE_MM;
        double yPosFirstMm, yPosLastMm, yStrideMm;
        if (iRow % 2 == 0)
        {
            yPosFirstMm = MOTION_STAGE_Y_MIN_PHYSICAL_MM;
            yStrideMm = CALIBRATION_SCAN_STRIDE_MM;
        }
        else
        {
            yPosFirstMm = MOTION_STAGE_Y_MIN_PHYSICAL_MM +
                          (numColumns - 1) * CALIBRATION_SCAN_STRIDE_MM;
            yStrideMm = -CALIBRATION_SCAN_STRIDE_MM;
        }
        for (int iCol = 0; iCol < numColumns; iCol++)
        {
            double xPosMm = yPosFirstMm + iCol * yStrideMm;
            calibrationPositions.push(std::make_tuple(yPosMm, xPosMm));
        }
    }
    spdlog::info("Calibration scan: {} positions to scan",
                 calibrationPositions.size());

    behaviorCamera.start();
    spdlog::info("Behavior camera started");

    auto [nextX, nextY] = calibrationPositions.front();
    double targetX = nextX;
    double targetY = nextY;
    calibrationPositions.pop();
    mine::axisPtrLookup_[mine::X_AXIS]->moveAbsolute(
        targetX,
        MOTION_STAGE_LENGTH_UNIT,
        false,
        CALIBRATION_SCAN_STAGE_SPEED,
        MOTION_STAGE_VELOCITY_UNIT);
    mine::axisPtrLookup_[mine::Y_AXIS]->moveAbsolute(
        targetY,
        MOTION_STAGE_LENGTH_UNIT,
        false,
        CALIBRATION_SCAN_STAGE_SPEED,
        MOTION_STAGE_VELOCITY_UNIT);
    spdlog::info("Moving to stage pos ({}, {})",
                 targetX, targetY);

    while (!calibrationPositions.empty())
    {
        FrameData frameData = behaviorCamera.waitForOneFrame();
        cv::Mat image = frameData.image;

        if (mine::axisPtrLookup_[mine::X_AXIS]->isBusy() ||
            mine::axisPtrLookup_[mine::Y_AXIS]->isBusy())
        {
            continue;
        }

        // Wait another 2 cycles to avoid aliasing
        frameData = behaviorCamera.waitForOneFrame();
        frameData = behaviorCamera.waitForOneFrame();

        cv::Mat rotatedImage;
        cv::rotate(image, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::Mat horizontalFlippedImage;
        cv::flip(rotatedImage, horizontalFlippedImage, 1); // dim 1 is horizontal)
        image = horizontalFlippedImage;

        // Save Image
        std::string filename = fmt::format(
            "aruco_scan_x{:.2f}_y{:.2f}.jpg", targetX, targetY);
        std::filesystem::path savePath = saveDir / filename;
        cv::imwrite(savePath.string(), image);

        spdlog::info("Image saved at stage pos ({}, {})",
                     targetX, targetY);

        // Move to next position
        auto [nextX, nextY] = calibrationPositions.front();
        targetX = nextX;
        targetY = nextY;
        calibrationPositions.pop();
        mine::axisPtrLookup_[mine::X_AXIS]->moveAbsolute(
            targetX,
            MOTION_STAGE_LENGTH_UNIT,
            false,
            CALIBRATION_SCAN_STAGE_SPEED,
            MOTION_STAGE_VELOCITY_UNIT);
        mine::axisPtrLookup_[mine::Y_AXIS]->moveAbsolute(
            targetY,
            MOTION_STAGE_LENGTH_UNIT,
            false,
            CALIBRATION_SCAN_STAGE_SPEED,
            MOTION_STAGE_VELOCITY_UNIT);
        spdlog::info("Moving to stage pos ({}, {})", targetX, targetY);
    }
}

int main()
{
    runCalibrationScan();
    return 0;
}