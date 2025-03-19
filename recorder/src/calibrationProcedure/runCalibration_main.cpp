#include "runCalibration.hpp"

namespace
{
    std::unique_ptr<BehaviorCamera> behaviorCamera = nullptr;

    std::queue<std::tuple<double, double>> getCalibrationPositions(
        double xMinMm,
        double xMaxMm,
        double yMinMm,
        double yMaxMm,
        double strideMm)
    {
        std::queue<std::tuple<double, double>> calibrationPositions;
        int numColumns = (xMaxMm - xMinMm) / strideMm + 1;
        int numRows = (yMaxMm - yMinMm) / strideMm + 1;
        for (int iCol = 0; iCol < numColumns; iCol++)
        {
            double xPosMm = xMinMm + iCol * strideMm;
            double yPosFirstMm, yStrideMm;
            if (iCol % 2 == 0)
            {
                yPosFirstMm = yMinMm;
                yStrideMm = strideMm;
            }
            else
            {
                yPosFirstMm = yMinMm + (numRows - 1) * strideMm;
                yStrideMm = -strideMm;
            }
            for (int iRow = 0; iRow < numRows; iRow++)
            {
                double yPosMm = yPosFirstMm + iRow * yStrideMm;
                calibrationPositions.push(std::make_tuple(xPosMm, yPosMm));
            }
        }
        return calibrationPositions;
    }

    void quitProgram()
    {
        spdlog::info("SIGINT received. Initiating graceful shutdown");
        if (behaviorCamera)
        {
            behaviorCamera->stop();
        }
        std::exit(0);
    }
}

void runCalibrationScan(RecorderConfig &recorderConfig,
                        std::filesystem::path arucoSaveDir)
{
    arucoSaveDir = prepareOutputFolder(arucoSaveDir, true);

    unsigned int imageWidth = roundToMultiplesOf64(
        recorderConfig.getParameter<int>("behavior_camera", "roi_width"));
    unsigned int imageHeight = roundToMultiplesOf64(
        recorderConfig.getParameter<int>("behavior_camera", "roi_height"));
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    std::string behaviorCameraFrameGrabberTriggerLine =
        recorderConfig.getParameter<std::string>(
            "behavior_camera", "frame_grabber_trigger_line");

    std::atomic<bool> cameraReadyFlag(false);
    behaviorCamera = std::make_unique<BehaviorCamera>(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        behaviorCameraFrameGrabberTriggerLine);

    spdlog::info("Behavior camera configured");

    MotionControl motionControl(recorderConfig);

    double stageXMinMm =
        recorderConfig.getParameter<double>("motion_control", "x_min_mm");
    double stageXMaxMm =
        recorderConfig.getParameter<double>("motion_control", "x_max_mm");
    double stageYMinMm =
        recorderConfig.getParameter<double>("motion_control", "y_min_mm");
    double stageYMaxMm =
        recorderConfig.getParameter<double>("motion_control", "y_max_mm");
    double calibrationScanStrideMm =
        recorderConfig.getParameter<double>("motion_control",
                                            "calibration_scan_stride_mm");
    std::queue<std::tuple<double, double>> calibrationPositions =
        getCalibrationPositions(stageXMinMm,
                                stageXMaxMm,
                                stageYMinMm,
                                stageYMaxMm,
                                calibrationScanStrideMm);
    spdlog::info("Calibration scan: {} positions to scan",
                 calibrationPositions.size());

    behaviorCamera->start();
    spdlog::info("Behavior camera started");

    // Move to first position
    auto [nextX, nextY] = calibrationPositions.front();
    double targetX = nextX;
    double targetY = nextY;
    calibrationPositions.pop();
    double motionVelocity = recorderConfig.getParameter<double>(
        "motion_control", "default_velocity_mm_per_sec");
    motionControl.moveAbsolute(X_AXIS, targetX, false, motionVelocity);
    motionControl.moveAbsolute(Y_AXIS, targetY, false, motionVelocity);
    spdlog::debug("Moving to stage pos ({}, {})", targetX, targetY);

    while (true)
    {
        FrameData frameData = behaviorCamera->waitForOneFrame();
        cv::Mat image = frameData.image;

        if (!motionControl.checkIfIdle(X_AXIS) ||
            !motionControl.checkIfIdle(Y_AXIS))
        {
            continue;
        }

        // Wait another 2 cycles to avoid aliasing
        frameData = behaviorCamera->waitForOneFrame();
        frameData = behaviorCamera->waitForOneFrame();

        cv::Mat rotatedImage;
        cv::rotate(image, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::Mat horizontalFlippedImage;
        cv::flip(rotatedImage, horizontalFlippedImage, 1); // dim 1 is horizontal)
        image = horizontalFlippedImage;

        // Save Image
        std::string filename = fmt::format(
            "aruco_scan_x{:.2f}_y{:.2f}.jpg", targetX, targetY);
        std::filesystem::path savePath = arucoSaveDir / filename;
        cv::imwrite(savePath.string(), image);

        spdlog::debug("Image saved at stage pos ({}, {})", targetX, targetY);

        // Move to next position
        if (calibrationPositions.empty())
        {
            spdlog::info("Calibration scan complete");
            break;
        }
        else
        {
            std::tie(targetX, targetY) = calibrationPositions.front();
            calibrationPositions.pop();
            motionControl.moveAbsolute(X_AXIS, targetX, false, motionVelocity);
            motionControl.moveAbsolute(Y_AXIS, targetY, false, motionVelocity);
            spdlog::debug("Moving to stage pos ({}, {})", targetX, targetY);
        }
    }

    behaviorCamera->stop();
    behaviorCamera.reset();
    spdlog::debug("Exiting calibration procedure");
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

    CLIOptions options = parseCLI(argc, argv);

    spdlog::set_level(options.logLevel);

    // Load recorder configuration
    std::filesystem::path profileDir =
        std::filesystem::path(expandPath(options.profileDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
    RecorderConfig recorderConfig(configPath);

    // Get save directory
    std::filesystem::path arucoSaveDir = profileDir / "calibration/aruco_scan";

    runCalibrationScan(recorderConfig, arucoSaveDir);
    spdlog::info("Calibration procedure complete");

    return 0;
}