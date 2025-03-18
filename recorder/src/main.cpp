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
    BehaviorRecordingState behaviorRecordingState;
}

// Define all global variables
// BehaviorCamera *behaviorCamera = nullptr;
// std::queue<GroupOfThreeFrames> behaviorImageQueue;
// std::mutex behaviorImageQueueMutex;
// std::condition_variable behaviorImageQueueCondVar;
// std::atomic<bool> behaviorCameraReady = false;
// std::queue<GroupOfThreeFrames> muscleImageQueue;
// std::mutex muscleImageQueueMutex;
// std::condition_variable muscleImageQueueCondVar;

std::atomic<bool> motionControlHandlerReady = false;
MotionStagePosition latestMotionStagePosition;
std::mutex latestMotionStagePositionMutex;
std::atomic<bool> isCalibrating = false;
std::atomic<bool> shouldOverrideTracking = false;
std::atomic<double> overrideXPosAbsolute;
std::atomic<double> overrideYPosAbsolute;

FrameData latestFrameData = {0, 0, 0, cv::Mat()};
std::mutex latestFrameMutex;

ArduinoTriggerControllerInterface *triggerController;

std::atomic<bool> toQuit = false;
std::atomic<bool> isRecording = false;
std::string saveDirectory = DEFAULT_SAVE_DIRECTORY;

QApplication *application = nullptr;
MainGUIWindow *mainGUIWindow = nullptr;

std::mutex isIOInitializing;

// Program control functions implementation
void initializeProgram()
{
    // Initialize application-wide resources and settings
    spdlog::info("Initializing application");

    // Reset global state flags
    toQuit.store(false);
    isRecording.store(false);

    // Other initialization code can be added here
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

    toQuit.store(true);

    // Stop behavior camera acquisition
    if (behaviorRecordingState.behaviorCamera)
    {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorRecordingState.behaviorCamera->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stopMotionControlRequestHandler();

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stopBehaviorImageSaver(behaviorRecordingState);

    std::exit(0);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

    // Set spdlog level to DEBUG
    spdlog::set_level(spdlog::level::debug);
    spdlog::debug("Debug level logging enabled");

    // Initialize program
    initializeProgram();

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Load recorder configuration
    std::filesystem::path configPath = expandPath(RECORDER_CONFIG_PATH);
    RecorderConfig recorderConfig(configPath);
    if (!recorderConfig.isDefined) {
        std::string errorMessage = fmt::format(
            "Failed to load recorder configuration. Cannot start recording. "
            "Expected valid recorder configuration file at {}",
            configPath.c_str());
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    // Load calibration parameters
    if (!updateCalibrationParams())
    {
        std::string errorMessage =
            "Calibration data do not exist. Cannot start recording.";
        spdlog::error(errorMessage);
        return 1;
    }

    // Start motion control IO thread
    std::thread motionControlIOThread(motionControlRequestHandler,
                                      recorderConfig);

    // Start motion stage position logger thread
    std::thread motionStagePositionLoggerThread(motionStagePositionLogger,
                                                recorderConfig);

    // Start tracking controller
    std::thread trackingControllerThread(trackingController,
                                         recorderConfig,
                                         std::ref(behaviorRecordingState));

    // Start behavior image acquirer
    std::thread behaviorImageAcquirerThread(behaviorImageAcquirer,
                                            std::ref(recorderConfig),
                                            std::ref(behaviorRecordingState));

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.push_back(
            std::thread(behaviorImageSaver, std::ref(behaviorRecordingState)));
    }

    // Start Arduino triggering interface
    ArduinoTriggerControllerInterface localTriggerController(recorderConfig);
    triggerController = &localTriggerController;

    // Create and show GUI
    MainGUIWindow localMainGUIWindow(recorderConfig,
                                     behaviorRecordingState,
                                     nullptr);
    mainGUIWindow = &localMainGUIWindow;
    mainGUIWindow->show();

    int result = application->exec();

    // Wait for threads to finish
    if (behaviorImageAcquirerThread.joinable())
    {
        behaviorImageAcquirerThread.join();
    }

    for (auto &thread : behaviorImageSaverThreads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    if (motionControlIOThread.joinable())
    {
        motionControlIOThread.join();
    }

    if (motionStagePositionLoggerThread.joinable())
    {
        motionStagePositionLoggerThread.join();
    }

    if (trackingControllerThread.joinable())
    {
        trackingControllerThread.join();
    }

    return result;
}