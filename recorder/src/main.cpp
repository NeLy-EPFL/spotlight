#include <memory>
#include <thread>
#include <vector>
#include <atomic>

#include <QApplication>

#include "global.hpp"
#include "recordingController.hpp"
#include "gui.hpp"

std::queue<GroupOfThreeFrames> behaviorImageQueue;
std::mutex behaviorImageQueueMutex;
std::condition_variable behaviorImageQueueCondVar;
std::atomic<bool> behaviorCameraReady = false;
std::queue<GroupOfThreeFrames> muscleImageQueue;
std::mutex muscleImageQueueMutex;
std::condition_variable muscleImageQueueCondVar;

// TODO: remove these if not needed
// std::mutex motionStageRequestMutex;
// std::condition_variable motionStageRequestCondVar;
// std::mutex motionStageResponseMutex;
// std::condition_variable motionStageResponseCondVar;
// std::atomic<bool> newRequestForMotionstage;
// std::atomic<bool> newPositionFromMotionStage;
// MotionStageRequest latestMotionStageRequest;
// MotionStagePosition latestMotionStagePosition;
std::atomic<bool> motionControlHandlerReady = false;

FrameData latestFrameData = {0, 0, 0, nullptr, false};
std::mutex latestFrameMutex;

std::atomic<bool> toQuit = false;
std::atomic<bool> isRecording = false;
std::string saveDirectory = DEFAULT_SAVE_DIRECTORY;

QApplication *application = nullptr;

namespace
{
    void handleSigint(int)
    {
        toQuit.store(true);
        if (application)
        {
            application->quit();
        }
    }
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, handleSigint);

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Start motion control IO thread
    std::thread motionControlIOThread(motionControlRequestHandler);

    // Start behavior image acquirer
    std::thread behaviorImageAcquiererThread(behaviorImageAcquierer);

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.push_back(std::thread(behaviorImageSaver));
    }

    // Create and show GUI
    MainGUIWindow MainGUIWindow(nullptr);
    MainGUIWindow.show();

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

    return result;
}