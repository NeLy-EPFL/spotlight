#include "runCalibration.hpp"

namespace
{
    std::unique_ptr<BehaviorCamera> behaviorCamera = nullptr;
    std::unique_ptr<MuscleCamera> muscleCamera = nullptr;

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
            behaviorCamera.reset();
        }
        std::exit(0);
    }

    bool waitUntilCamerasReady(int maxWaitTimeSec = 10)
    {
        for (int sec = 0; sec < maxWaitTimeSec; sec++)
        {
            FrameData behaviorFrameData = behaviorCamera->waitForOneFrame();
            FrameData muscleFrameData = muscleCamera->waitForOneFrame();
            bool behaviorCameraReady = !behaviorFrameData.image.empty();
            bool muscleCameraReady = !muscleFrameData.image.empty();
            if (behaviorCameraReady && muscleCameraReady)
            {
                spdlog::info("Cameras are ready");
                return true;
            }
            else
            {
                spdlog::warn("Waiting for cameras to be ready...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        spdlog::critical("Cameras are not ready after {} seconds",
                         maxWaitTimeSec);
        return false;
    }
}

void runCalibrationScan(std::filesystem::path profileDir,
                        std::filesystem::path arucoSaveDir)
{
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
    RecorderConfig recorderConfig(configPath);

    arucoSaveDir = prepareOutputFolder(arucoSaveDir, true);

    // Set up behavior camera
    spdlog::info("Configuring behavior camera");
    BehaviorCameraROI behaviorCameraROI =
        getBehaviorBehaviorCameraROI(recorderConfig);
    std::string behaviorCameraFrameGrabberTriggerLine =
        recorderConfig.getParameter<std::string>(
            "behavior_camera", "frame_grabber_trigger_line");

    std::atomic<bool> behaviorCameraReadyFlag(false);
    behaviorCamera = std::make_unique<BehaviorCamera>(
        behaviorCameraROI.imageWidth,
        behaviorCameraROI.imageHeight,
        behaviorCameraROI.xOffset,
        behaviorCameraROI.yOffset,
        behaviorCameraFrameGrabberTriggerLine);
    spdlog::info("Behavior camera configured");

    // Set up muscle camera
    spdlog::info("Configuring muscle camera");
    MuscleCameraROI muscleCameraROI = getMuscleCameraROI(
        profileDir / "muscle_camera_roi.yaml");
    spdlog::info(
        "width: {}, height: {}, xOffset: {}, yOffset: {}",
        muscleCameraROI.x1 - muscleCameraROI.x0 + 1,
        muscleCameraROI.y1 - muscleCameraROI.y0 + 1,
        muscleCameraROI.x0 - 1,
        muscleCameraROI.y0 - 1);
    muscleCamera = std::make_unique<MuscleCamera>(
        muscleCameraROI.x1 - muscleCameraROI.x0 + 1,
        muscleCameraROI.y1 - muscleCameraROI.y0 + 1,
        muscleCameraROI.x0 - 1,
        muscleCameraROI.y0 - 1,
        recorderConfig,
        profileDir.string(),
        spdlog::get_level());
    spdlog::info("Muscle camera configured");

    spdlog::info(
        "Waiting for cameras to be ready. This may take a few seconds...");
    waitUntilCamerasReady();

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

    FrameData behaviorFrameData;
    FrameData muscleFrameData;
    cv::Mat behaviorImage;
    cv::Mat muscleImage;
    while (true)
    {
        if (!motionControl.checkIfIdle(X_AXIS) ||
            !motionControl.checkIfIdle(Y_AXIS))
        {
            continue;
        }

        // Wait another 2 cycles to avoid aliasing
        behaviorFrameData = behaviorCamera->waitForOneFrame();
        behaviorFrameData = behaviorCamera->waitForOneFrame();
        muscleFrameData = behaviorCamera->waitForOneFrame();
        muscleFrameData = behaviorCamera->waitForOneFrame();

        behaviorImage = reorientBehaviorImage(behaviorFrameData.image);
        muscleImage = reorientMuscleImage(muscleFrameData.image);

        // Save Image
        std::string filename = fmt::format(
            "aruco_scan_x{:.2f}_y{:.2f}.jpg", targetX, targetY);
        cv::imwrite((arucoSaveDir / "behaviorCamera" / filename).string(),
                    behaviorImage);
        cv::imwrite((arucoSaveDir / "muscleCamera" / filename).string(),
                    muscleImage);

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

    // Get save directory
    std::filesystem::path arucoSaveDir = profileDir / "calibration/aruco_scan";

    runCalibrationScan(profileDir, arucoSaveDir);
    spdlog::info("Calibration procedure complete");

    return 0;
}