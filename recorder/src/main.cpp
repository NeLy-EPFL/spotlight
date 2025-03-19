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
#include "behaviorRecording.hpp"
#include "muscleRecording.hpp"
#include "trackingControl.hpp"
#include "gui.hpp"

namespace
{
    QApplication *application = nullptr;
    MainGUIWindow *mainGUIWindow = nullptr;
    std::shared_ptr<ProgramState> programState;
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState;
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

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stopMotionControlRequestHandler(programState);

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stopBehaviorImageSaver(behaviorRecordingState, programState);

    std::exit(0);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

    // Set spdlog level to DEBUG
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("Debug level logging enabled");

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Make program state holder
    programState = std::make_shared<ProgramState>();
    behaviorRecordingState = std::make_shared<BehaviorRecordingState>();

    // Load recorder configuration
    std::filesystem::path configPath = expandPath(RECORDER_CONFIG_PATH);
    spdlog::info("Loading recorder configuration from {}", configPath.c_str());
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
        recorderConfig.getParameter<std::string>("io", "default_save_dir");
    // SaveDirectory saveDirectory(defaultSaveDirectory);
    std::shared_ptr<SaveDirectory> saveDirectory =
        std::make_shared<SaveDirectory>(defaultSaveDirectory);

    // Load position mapping/calibration parameters
    std::string calibrationParamsFilePath =
        recorderConfig.getParameter<std::string>("io", "calibration_file");
    calibrationParamsFilePath = expandPath(calibrationParamsFilePath);
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

    // Initialize holder for latest frame data (used for live streaming) and
    // fly tracking
    LatestFrame latestBehaviorFrameHolder;

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
        std::ref(recorderConfig),
        trackingControlState,
        saveDirectory,
        programState);
    std::thread trackingControllerThread(
        trackingController,
        recorderConfig,
        behaviorRecordingState,
        trackingControlState,
        std::ref(behaviorCamCalibrationParams),
        std::ref(latestBehaviorFrameHolder),
        programState);

    // Start behavior image acquirer
    std::thread behaviorImageAcquirerThread(
        behaviorImageAcquirer,
        std::ref(recorderConfig),
        behaviorRecordingState,
        std::ref(latestBehaviorFrameHolder),
        programState);

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.push_back(
            std::thread(behaviorImageSaver,
                        recorderConfig,
                        behaviorRecordingState,
                        saveDirectory,
                        programState));
    }

    // Start Arduino triggering interface
    std::shared_ptr<ArduinoTriggerInterface> arduinoTriggerInterface =
        std::make_shared<ArduinoTriggerInterface>(recorderConfig, programState);

    // Create and show GUI
    MainGUIWindow localMainGUIWindow(recorderConfig,
                                     behaviorRecordingState,
                                     trackingControlState,
                                     std::ref(behaviorCamCalibrationParams),
                                     saveDirectory,
                                     latestBehaviorFrameHolder,
                                     arduinoTriggerInterface,
                                     nullptr);
    mainGUIWindow = &localMainGUIWindow;
    mainGUIWindow->show();

    int result = application->exec();

    // Wait for threads to finish
    spdlog::debug("Waiting for threads to finish");
    if (behaviorImageAcquirerThread.joinable())
    {
        behaviorImageAcquirerThread.join();
    }
    spdlog::debug("Behavior image acquirer thread finished");

    for (auto &thread : behaviorImageSaverThreads)
    {
        spdlog::debug("Waiting for behavior image saver thread to finish");
        if (thread.joinable())
        {
            thread.join();
        }
        spdlog::debug("Behavior image saver thread finished");
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