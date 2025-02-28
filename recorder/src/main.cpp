#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <signal.h>
#include <csignal>

#include <QApplication>
#include <spdlog/spdlog.h>

#include "main.hpp"
#include "behaviorRecording.hpp"
#include "muscleRecording.hpp"
#include "trackingControl.hpp"
#include "gui.hpp"

// Define all global variables
BehaviorCamera *behaviorCamera = nullptr;
std::queue<GroupOfThreeFrames> behaviorImageQueue;
std::mutex behaviorImageQueueMutex;
std::condition_variable behaviorImageQueueCondVar;
std::atomic<bool> behaviorCameraReady = false;
std::queue<GroupOfThreeFrames> muscleImageQueue;
std::mutex muscleImageQueueMutex;
std::condition_variable muscleImageQueueCondVar;

std::atomic<bool> motionControlHandlerReady = false;
MotionStagePosition latestMotionStagePosition;
std::mutex latestMotionStagePositionMutex;

FrameData latestFrameData = {0, 0, 0, nullptr};
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
 * * Without stopping acquiisition explicitly, the frame grabber will
 * think the device is still busy the next time we run the program.
 */
{
    spdlog::info("SIGINT received. Initiating graceful shutdown");

    if (!mainGUIWindow->canQuitGracefully())
    {
        std::string errorMessage =
            "Cannot quit gracefully because something is still running in "
            "the background. Retry later or force quit (but then you "
            "should manually reset camera acquisition state).";
        spdlog::error(errorMessage);
        QMessageBox::critical(
            mainGUIWindow, "Error", QString(errorMessage.c_str()));
        return false;
    }

    toQuit.store(true);

    // Stop behavior camera acquisition
    if (behaviorCamera)
    {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorCamera->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stopMotionControlRequestHandler();

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.",
                 NUM_BEHAVIOR_IMAGE_SAVING_THREADS);
    stopBehaviorImageSaver();

    std::exit(0);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

    // Initialize program
    initializeProgram();

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Start motion control IO thread
    std::thread motionControlIOThread(motionControlRequestHandler);

    // Start motion stage position logger thread
    std::thread motionStagePositionLoggerThread(motionStagePositionLogger);

    // Start behavior image acquirer
    std::thread behaviorImageAcquiererThread(behaviorImageAcquierer);

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.push_back(std::thread(behaviorImageSaver));
    }

    // Start Arduino triggering interface
    ArduinoTriggerControllerInterface localTriggerController;
    triggerController = &localTriggerController;

    // Create and show GUI
    MainGUIWindow localMainGUIWindow(nullptr);
    mainGUIWindow = &localMainGUIWindow;
    mainGUIWindow->show();

    int result = application->exec();

    // Wait for threads to finish
    if (behaviorImageAcquiererThread.joinable())
    {
        behaviorImageAcquiererThread.join();
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

    return result;
}