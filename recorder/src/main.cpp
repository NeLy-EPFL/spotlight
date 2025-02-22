#include <memory>
#include <thread>
#include <vector>
#include <atomic>

#include <QApplication>

#include "global.hpp"
#include "recordingController.hpp"
#include "gui.hpp"

std::queue<FrameData> behaviorImageQueue;
std::mutex behaviorImageQueueMutex;
std::condition_variable behaviorImageQueueCondVar;
std::queue<FrameData> muscleImageQueue;
std::mutex muscleImageQueueMutex;
std::condition_variable muscleImageQueueCondVar;
FrameData latestFrameData = {0, 0, nullptr, false};
std::mutex latestFrameMutex;

std::shared_ptr<std::atomic<bool>> toQuit;
std::shared_ptr<std::atomic<bool>> isRecording;

QApplication *application = nullptr;

namespace {
    void handleSigint(int)
    {
        toQuit->store(true);
        if (application)
        {
            application->quit();
        }
    }
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, handleSigint);

    isRecording = std::make_shared<std::atomic<bool>>(false);
    toQuit = std::make_shared<std::atomic<bool>>(false);

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Start behavior image acquirer
    std::thread behaviorImageAcquiererThread(behaviorImageAcquierer);

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.emplace_back(behaviorImageSaver, "./images");
    }

    // Create and show GUI
    Gui gui(isRecording, toQuit);
    gui.show();

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