#include <memory>
#include <thread>
#include <vector>
#include <atomic>

#include <QApplication>

#include "recordingController.hpp"
#include "gui.hpp"

std::shared_ptr<std::atomic<bool>> toQuit;
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

    std::shared_ptr<std::atomic<bool>> isSavingData =
        std::make_shared<std::atomic<bool>>(false);
    toQuit = std::make_shared<std::atomic<bool>>(false);

    QApplication localApplication(argc, argv);
    application = &localApplication;

    // Start behavior image acquirer
    std::thread behaviorImageAcquiererThread(
        behaviorImageAcquierer, isSavingData, toQuit);

    // Start behavior image saver
    std::vector<std::thread> behaviorImageSaverThreads;
    for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
    {
        behaviorImageSaverThreads.emplace_back(
            behaviorImageSaver, "./images", isSavingData, toQuit);
    }

    // Create and show GUI
    Gui gui(isSavingData, toQuit);
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