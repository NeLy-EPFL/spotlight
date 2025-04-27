#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <signal.h>
#include <csignal>
#include <filesystem>

#include <QApplication>
#include <spdlog/spdlog.h>

#include "main.hpp"

namespace
{
    QApplication *application = nullptr;
    MainGUIWindow *mainGUIWindow = nullptr;
    std::shared_ptr<ProgramState> programState;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState;
    std::shared_ptr<MuscleRecordingState> muscleRecordingState;
    std::shared_ptr<ArduinoCommunication> arduinoCommunication;
}

bool quitProgram()
/**
 * Quit gracefully by explicitly stopping acquisition on the behavior
 * camera* and telling saver threads that the work is done.
 *
 * * Without stopping acquisition explicitly, the frame grabber will
 * think the device is still busy the next time we run the program.
 */
{
    spdlog::info("SIGINT received. Initiating graceful shutdown");

    programState->toQuit.store(true);

    // Stop behavior camera acquisition
    if (behaviorRecordingState->behaviorCamera)
    {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorRecordingState->behaviorCamera->stop();
    }

    // Terminate PCO camera server
    if (muscleRecordingState->muscleCamera)
    {
        spdlog::info("Stopping acquisition on muscle camera");
        pid_t pcoCameraServerPid =
            muscleRecordingState->muscleCamera->getCameraServerPID();
        kill(pcoCameraServerPid, SIGTERM);
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stopMotionControlRequestHandler(programState);

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stopBehaviorImageSaver(behaviorRecordingState, programState);
    muscleRecordingState->muscleCamera = nullptr;

    spdlog::info("Telling muscle image saver threads to stop.");
    stopMuscleImageSaver(muscleRecordingState, programState);

    // Stop muscle excitation
    spdlog::info("Switching off muscle excitation light.");
    arduinoCommunication->setSyncRatio(INT_MAX);

    std::exit(0);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

    // Parse command line arguments
    CLIOptions options = parseCLI(argc, argv);

    // Set log level based on CLI options
    spdlog::set_level(options.logLevel);

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Make program state holder
    programState = std::make_shared<ProgramState>();

    // Load recorder configuration
    std::filesystem::path profileDir =
        std::filesystem::path(expandPath(options.profileDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}", configPath.string());
    RecorderConfig recorderConfig(configPath);
    if (!recorderConfig.isDefined)
    {
        std::string errorMessage = fmt::format(
            "Failed to load recorder configuration. Cannot start recording. "
            "Expected valid recorder configuration file at {}",
            configPath.c_str());
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    // Make atomic variable that holds the save directory
    std::string defaultSaveDirectory =
        recorderConfig.getParameter<std::string>("gui", "default_save_dir");
    // SaveDirectory saveDirectory(defaultSaveDirectory);
    std::shared_ptr<SaveDirectory> saveDirectory =
        std::make_shared<SaveDirectory>(defaultSaveDirectory);

    // Load position mapping/calibration parameters
    std::string calibrationParamsFilePath =
        profileDir / "calibration/calibration_result.yaml";
    spdlog::info("Loading spatial calibration parameters from {}",
                 calibrationParamsFilePath);
    CalibrationParams behaviorCamCalibrationParams(calibrationParamsFilePath);
    if (!behaviorCamCalibrationParams.isDefined)
    {
        std::string errorMessage = fmt::format(
            "Spatial calibration data not found or malformed. This is required "
            "for tracking and recording. Expected valid calibration file at {} "
            "based on the recorder configuration file.",
            calibrationParamsFilePath);
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    // Initialize behavior and muscle imaging states
    behaviorRecordingState = std::make_shared<BehaviorRecordingState>();
    behaviorRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();
    muscleRecordingState = std::make_shared<MuscleRecordingState>();
    muscleRecordingState->latestFrameHolder = std::make_shared<LatestFrame>();

    // Start tracking & motion control threads
    std::shared_ptr<TrackingControlState> trackingControlState =
        std::make_shared<TrackingControlState>();
    std::thread motionControlIOThread(
        motionControlRequestHandler,
        recorderConfig,
        trackingControlState,
        programState);
    std::thread motionStagePositionLoggerThread(
        motionStagePositionLogger,
        recorderConfig,
        trackingControlState,
        saveDirectory,
        programState);
    std::thread trackingControllerThread(
        trackingController,
        recorderConfig,
        behaviorRecordingState,
        trackingControlState,
        std::ref(behaviorCamCalibrationParams),
        programState);

    // Start behavior image acquirer
    std::shared_ptr<ProgrammedStop> programmedRecordingStop =
        std::make_shared<ProgrammedStop>();

    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        recorderConfig,
        behaviorRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Behavior camera acquisition thread started");

    // Start behavior image savers
    std::vector<std::thread> behaviorImageSaverThreads;
    int numBehaviorImageSaverThreads =
        recorderConfig.getParameter<int>("behavior_camera",
                                         "num_image_saving_threads");
    for (int i = 0; i < numBehaviorImageSaverThreads; i++)
    {
        behaviorImageSaverThreads.push_back(
            std::thread(behaviorImageSaver,
                        recorderConfig,
                        behaviorRecordingState,
                        saveDirectory,
                        programState));
    }
    spdlog::info("Behavior camera saver threads started");

    // Start muscle image acquirer
    std::filesystem::path roiFilePath = profileDir / "muscle_camera_roi.yaml";
    MuscleCameraROI muscleROI = getMuscleCameraROI(roiFilePath);
    spdlog::info(
        "Loaded muscle camera ROI from {}: x0={}, x1={}, y0={}, y1={} "
        "(xOffset={}, yOffset={}, imageWidth={}, imageHeight={})",
        muscleROI.x0, muscleROI.x1, muscleROI.y0, muscleROI.y1,
        muscleROI.xOffset, muscleROI.yOffset,
        muscleROI.imageWidth, muscleROI.imageHeight);
    std::thread muscleImageAcquirerThread(
        muscleImageAcquierer,
        muscleROI.imageWidth,
        muscleROI.imageHeight,
        muscleROI.xOffset,
        muscleROI.yOffset,
        recorderConfig,
        profileDir,
        spdlog::get_level(),
        muscleRecordingState,
        programState,
        programmedRecordingStop);
    spdlog::info("Muscle camera acquisition thread started");

    // Start muscle image savers
    std::vector<std::thread> muscleImageSaverThreads;
    int numMuscleImageSaverThreads =
        recorderConfig.getParameter<int>("muscle_camera",
                                         "num_image_saving_threads");
    for (int i = 0; i < numMuscleImageSaverThreads; i++)
    {
        muscleImageSaverThreads.push_back(
            std::thread(muscleImageSaver,
                        recorderConfig,
                        muscleRecordingState,
                        saveDirectory,
                        programState,
                        5)); // cv::IMWRITE_TIFF_COMPRESSION_LZW
    }
    spdlog::info("Muscle camera saver threads started");

    // Start Arduino triggering interface
    std::string arduinoPortName = findArduinoPortName(recorderConfig);
    arduinoCommunication =
        std::make_shared<ArduinoCommunication>(arduinoPortName);

    // Create and show GUI
    MainGUIWindow localMainGUIWindow(recorderConfig,
                                     behaviorRecordingState,
                                     muscleRecordingState,
                                     trackingControlState,
                                     std::ref(behaviorCamCalibrationParams),
                                     saveDirectory,
                                     arduinoCommunication,
                                     programState,
                                     programmedRecordingStop,
                                     nullptr);
    mainGUIWindow = &localMainGUIWindow;
    mainGUIWindow->show();

    int result = application->exec();

    // Wait for threads to finish
    spdlog::debug("Waiting for behavior image acquirer thread to finish");
    if (behaviorImageAcquirerThread.joinable())
    {
        behaviorImageAcquirerThread.join();
    }
    spdlog::debug("Behavior image acquirer thread finished");

    for (auto &thread : behaviorImageSaverThreads)
    {
        spdlog::debug(
            "Waiting for one of the behavior image saver threads to finish");
        if (thread.joinable())
        {
            thread.join();
        }
        spdlog::debug("One of the behavior image saver threads finished");
    }

    spdlog::debug("Waiting for muscle image acquirer thread to finish");
    if (muscleImageAcquirerThread.joinable())
    {
        muscleImageAcquirerThread.join();
    }
    spdlog::debug("Muscle image acquirer thread finished");

    for (auto &thread : muscleImageSaverThreads)
    {
        spdlog::debug(
            "Waiting for one of the muscle image saver threads to finish");
        if (thread.joinable())
        {
            thread.join();
        }
        spdlog::debug("One of the muscle image saver threads finished");
    }

    spdlog::debug("Waiting for motion control IO thread to finish");
    if (motionControlIOThread.joinable())
    {
        motionControlIOThread.join();
    }
    spdlog::debug("Motion control IO thread finished");

    spdlog::debug("Waiting for motion stage position logger thread to finish");
    if (motionStagePositionLoggerThread.joinable())
    {
        motionStagePositionLoggerThread.join();
    }
    spdlog::debug("Motion stage position logger thread finished");

    spdlog::debug("Waiting for tracking controller thread to finish");
    if (trackingControllerThread.joinable())
    {
        trackingControllerThread.join();
    }
    spdlog::debug("Tracking controller thread finished");

    return result;
}