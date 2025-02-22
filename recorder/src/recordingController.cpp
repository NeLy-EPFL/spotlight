#include "recordingController.hpp"

namespace
{
    BehaviorCamera *behaviorCamera = nullptr;

    std::atomic<bool> isRecording(false);
    uint frameNumber = 0;

    void handleSigint(int)
    /**
     * Handle SIGINT signal: quit gracefully by explicitly stopping
     * acquisition on the behavior camera. Without this, acquisition would
     * technically never stops. Consequently, the frame grabber will think
     * the device is busy the next time we run the program.
     */
    {
        spdlog::info("SIGINT received by behavior camera acquisition thread. "
                     "eGrabber closing acquisition");

        // Stop behavior camera acquisition
        if (behaviorCamera)
        {
            behaviorCamera->stop();
        }

        // Tell behavior camera saver threads to stop
        spdlog::info("Telling behavior image saver threads to stop by adding "
                     "{} stoppers to behavior image queue",
                     NUM_BEHAVIOR_IMAGE_SAVING_THREADS);
        for (int i = 0; i < NUM_BEHAVIOR_IMAGE_SAVING_THREADS; i++)
        {
            FrameData stopper;
            stopper.noMoreData = true;
            {
                std::lock_guard<std::mutex> lock(behaviorImageQueueMutex);
                behaviorImageQueue.push(stopper);
            }
            behaviorImageQueueCondVar.notify_one();
        }

        std::exit(0);
    }
}

void behaviorImageAcquierer()
{
    std::signal(SIGINT, handleSigint);

    unsigned int imageWidth = roundToMultiplesOf64(
        BEHAVIOR_CAMERA_ROI_WIDTH);
    unsigned int imageHeight = roundToMultiplesOf64(
        BEHAVIOR_CAMERA_ROI_HEIGHT);
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    BehaviorCamera localBehaviorCamera(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        BEHAVIOR_CAMERA_FRAME_GRABBER_TRIGGER_LINE);
    behaviorCamera = &localBehaviorCamera;

    spdlog::info("Behavior camera configured");

    behaviorCamera->start();
    spdlog::info("Behavior camera started");

    FrameData frameData;

    while (!toQuit->load())
    {
        // Acquire image data
        frameData = behaviorCamera->waitForOneFrame();

        // Add to queue
        {
            std::lock_guard<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueue.push(frameData);
        }
        behaviorImageQueueCondVar.notify_one();
        {
            {
                std::lock_guard<std::mutex> lock(latestFrameMutex);
                std::swap(latestFrameData, frameData);
            }
        }
    }

    behaviorCamera->stop();
    spdlog::info("eGrabber acquisition stopped; "
                 "behavior camera acquisition thread stopped");
}

void behaviorImageSaver(const std::string &directory)
{
    std::string filename;
    FrameData frameData;
    uint currentFrameNumber = frameNumber;

    while (!toQuit->load())
    {
        {
            std::unique_lock<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueueCondVar.wait(
                lock, []
                { return !behaviorImageQueue.empty(); });
            frameData = behaviorImageQueue.front();
            behaviorImageQueue.pop();
            currentFrameNumber = frameNumber++;
        }

        if (frameData.noMoreData)
        {
            spdlog::info("Behavior image saver received stopper; "
                         "no more frame will come.");
            break;
        }

        filename = directory +
                   "/behavior_" +
                   fmt::format("{:09}", currentFrameNumber) +
                   ".tiff";
        // cv::imwrite(filename, *frameData.imagePtr);
        // spdlog::info("Behavior image would be saved to {}", filename);
    }
    spdlog::info("Behavior image saver thread stopped");
}