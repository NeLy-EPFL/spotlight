#include "runCalibration.hpp"

namespace
{
    std::shared_ptr<ProgramState> programState = nullptr;
    std::shared_ptr<ProgrammedStop> programmedRecordingStop = nullptr;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState = nullptr;
    std::shared_ptr<MuscleRecordingState> muscleRecordingState = nullptr;
    std::shared_ptr<ArduinoCommunication> arduinoCommunication = nullptr;

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
        if (behaviorRecordingState)
        {
            behaviorRecordingState->behaviorCamera->stop();
        }

        // Stop triggering
        spdlog::info("Stopping triggering via Arduino");
        arduinoCommunication->setBehaviorRecordingFPS(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduinoCommunication->stopCommunication();
        
        std::exit(0);
    }
}

void runCalibrationScan(std::filesystem::path profileDir,
                        std::filesystem::path arucoSaveDir)
{
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
    RecorderConfig recorderConfig(configPath);

    arucoSaveDir = prepareOutputFolder(arucoSaveDir, true);
    prepareOutputFolder(arucoSaveDir / "behavior_camera", true);
    prepareOutputFolder(arucoSaveDir / "muscle_camera", true);

    // Load muscle camera ROI
    std::filesystem::path roiFilePath = profileDir / "muscle_camera_roi.yaml";
    MuscleCameraROI muscleROI = getMuscleCameraROI(roiFilePath);
    spdlog::info(
        "Loaded muscle camera ROI from {}: x0={}, x1={}, y0={}, y1={} "
        "(xOffset={}, yOffset={}, imageWidth={}, imageHeight={})",
        muscleROI.x0, muscleROI.x1, muscleROI.y0, muscleROI.y1,
        muscleROI.xOffset, muscleROI.yOffset,
        muscleROI.imageWidth, muscleROI.imageHeight);

    // Set up shared recording states
    programState = std::make_shared<ProgramState>();
    programmedRecordingStop = std::make_shared<ProgrammedStop>();
    behaviorRecordingState = std::make_shared<BehaviorRecordingState>();
    muscleRecordingState = std::make_shared<MuscleRecordingState>();

    // Set up cameras acquisition threads
    spdlog::info("Starting behavior camera acquisition thread");
    behaviorRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Behavior camera acquisition thread started");

    spdlog::info("Setting up muscle camera acquisition thread");
    muscleRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    std::thread muscleImageAcquirerThread(
        muscleImageAcquirer,
        muscleROI.imageWidth,
        muscleROI.imageHeight,
        muscleROI.xOffset,
        muscleROI.yOffset,
        0,  // delay after trigger
        recorderConfig,
        profileDir,
        spdlog::get_level(),
        muscleRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Muscle camera acquisition thread started");

    // Start Arduino triggering interface set default triggering parameters
    while (!muscleRecordingState->muscleCamera)
    {
        spdlog::debug("Waiting for muscle camera to be ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    int muscleNumLinesScanned =
        muscleRecordingState->muscleCamera->getNumLinesScanned();
    arduinoCommunication = initializeTriggeringWithDefaultParams(
        recorderConfig, muscleNumLinesScanned, 1);

    // Set up motion control
    MotionControl motionControl(recorderConfig);

    // Figure out which points to park at
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
    int64_t firstBehaviorReceivedTime;
    int64_t firstMuscleReceivedTime;
    while (true)
    {
        if (!motionControl.checkIfIdle(X_AXIS) ||
            !motionControl.checkIfIdle(Y_AXIS))
        {
            continue;
        }

        // Take the SECOND image that arrives since the motion stages are idle
        // This is because the stages might still be moving when the first image
        // was exposed.
        behaviorFrameData = behaviorRecordingState
                                ->latestFrameHolder
                                ->getLatestFrameData();
        muscleFrameData = muscleRecordingState
                              ->latestFrameHolder
                              ->getLatestFrameData();
        firstBehaviorReceivedTime = behaviorFrameData.receivedTime;
        firstMuscleReceivedTime = muscleFrameData.receivedTime;

        while (firstBehaviorReceivedTime == behaviorFrameData.receivedTime ||
               firstMuscleReceivedTime == muscleFrameData.receivedTime)
        {
            behaviorFrameData = behaviorRecordingState
                                    ->latestFrameHolder
                                    ->getLatestFrameData();
            muscleFrameData = muscleRecordingState
                                  ->latestFrameHolder
                                  ->getLatestFrameData();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        reorientBehaviorImage(behaviorFrameData.image, behaviorImage);
        reorientMuscleImage(muscleFrameData.image, muscleImage);

        // Save Image
        std::filesystem::path behaviorPath =
            arucoSaveDir /
            "behavior_camera" /
            fmt::format("aruco_scan_x{:.2f}_y{:.2f}.jpg", targetX, targetY);
        cv::imwrite(behaviorPath.string(), behaviorImage);
        std::filesystem::path musclePath =
            arucoSaveDir /
            "muscle_camera" /
            fmt::format("aruco_scan_x{:.2f}_y{:.2f}.tif", targetX, targetY);
        cv::imwrite(musclePath.string(), muscleImage);
        spdlog::debug("Image saved at stage position ({}, {})",
                      targetX, targetY);

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

    // Stop the cameras
    spdlog::info("Stopping behavior camera acquisition thread");
    programState->toQuit.store(true);
    // Give some time for acquisition threads to break out of loop
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (behaviorRecordingState->behaviorCamera)
    {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorRecordingState->behaviorCamera->stop();
        behaviorRecordingState->behaviorCamera = nullptr;
    }
    muscleRecordingState->muscleCamera = nullptr;
    behaviorImageAcquirerThread.join();
    muscleImageAcquirerThread.join();

    // Stop triggering
    spdlog::info("Stopping triggering via Arduino");
    arduinoCommunication->setBehaviorRecordingFPS(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arduinoCommunication->stopCommunication();
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