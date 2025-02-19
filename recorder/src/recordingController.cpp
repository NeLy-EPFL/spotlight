#include "recordingController.hpp"

namespace
{
    std::queue<FrameData> behaviorImageQueue;
    std::mutex behaviorImageQueueMutex;
    std::condition_variable behaviorImageQueueCondVar;
    std::queue<FrameData> muscleImageQueue;
    std::mutex muscleImageQueueMutex;
    std::condition_variable muscleImageQueueCondVar;

    std::atomic<bool> isRecording(false);
    std::atomic<bool> isDone(false);
    uint frameNumber = 0;
}

void behaviorImageAcquierer(std::shared_ptr<bool> isSavingData)
{
    unsigned int imageWidth = roundToMultiplesOf64(640);
    unsigned int imageHeight = roundToMultiplesOf64(480);
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    BehaviorCamera behaviorCamera(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        BEHAVIOR_CAMERA_FRAME_GRABBER_TRIGGER_LINE);
    spdlog::info("Behavior camera configured");

    behaviorCamera.start();
    spdlog::info("Behavior camera started");

    FrameData frameData;

    while (!isDone)
    {
        // Acquire image data
        frameData = behaviorCamera.waitForOneFrame();

        // Add to queue
        {
            std::lock_guard<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueue.push(frameData);
        }
        behaviorImageQueueCondVar.notify_one();
    }
}

void behaviorImageSaver(
    const std::string &directory, std::shared_ptr<bool> isSavingData)
{
    std::string filename;
    FrameData frameData;
    uint currentFrameNumber = frameNumber;

    while (!isDone)
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

        filename = directory +
                   "/behavior_" +
                   fmt::format("{:09}", currentFrameNumber) +
                   ".tiff";
        // cv::imwrite(filename, *frameData.imagePtr);
        spdlog::info("Behavior image would be saved to {}", filename);
    }
}