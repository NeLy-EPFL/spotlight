#include "recordingController.hpp"

namespace
{
    BehaviorCamera *behaviorCamera = nullptr;

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
            spdlog::info("Stopping acquisition on behavior camera");
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
            // The behavior image queue actually keeps track of buffer groups
            // of three images, so we just fill the buffer with stoppers here
            GroupOfThreeFrames stopperBlock = {stopper, stopper, stopper};
            {
                std::lock_guard<std::mutex> lock(behaviorImageQueueMutex);
                behaviorImageQueue.push(stopperBlock);
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

    FrameData frameDataBuffer[3];
    size_t frameDataBufferIndex = 0;
    long int currentFrameId = 0;

    while (!toQuit->load())
    {
        // Acquire image data
        FrameData frameData = behaviorCamera->waitForOneFrame();

        // Update latest frame for live display
        {
            std::lock_guard<std::mutex> lock(latestFrameMutex);
            std::swap(latestFrameData, frameData);
        }

        if (isRecording->load())
        {
            // Assign frame ID only if recording. This way, for each recording
            // session, the frame ID starts from 0 regardless of how many
            // images the programs has received globally.
            frameData.frameId = currentFrameId++;

            frameDataBuffer[frameDataBufferIndex++] = frameData;

            if (frameDataBufferIndex == 3)
            {
                // Add to queue
                GroupOfThreeFrames groupOfThreeFrames = {
                    frameDataBuffer[0],
                    frameDataBuffer[1],
                    frameDataBuffer[2]};
                {
                    std::lock_guard<std::mutex> lock(behaviorImageQueueMutex);
                    behaviorImageQueue.push(groupOfThreeFrames);
                }
                behaviorImageQueueCondVar.notify_one();

                frameDataBufferIndex = 0;
            }
        }
        else
        {
            // Reset these to 0 in preparation for the next recording session
            frameDataBufferIndex = 0;
            currentFrameId = 0;
        }
    }
}

void behaviorImageSaver()
{
    while (!toQuit->load())
    {
        GroupOfThreeFrames frameGroup;
        {
            std::unique_lock<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueueCondVar.wait(
                lock, []
                { return !behaviorImageQueue.empty(); });
            frameGroup = behaviorImageQueue.front();
            behaviorImageQueue.pop();
        }

        if (frameGroup.frame0.noMoreData)
        {
            spdlog::info("Behavior image saver received stopper; "
                         "no more frame will come.");
            break;
        }

        std::string filenameStem =
            saveDirectory +
            "/behavior_" +
            fmt::format("{:09}", frameGroup.frame0.frameId);

        // Save three frames as a single pseudo-RGB image
        std::string filename = filenameStem + ".jpg";
        cv::Mat image = makePseudoRGBImageFromThreeFrames(frameGroup);
        cv::imwrite(filename, image);

        // Save metadata
        std::string metadataFilename = filenameStem + ".txt";
        std::ofstream metadataFile(metadataFilename);
        metadataFile << makeMetadataStringFromThreeFrames(frameGroup);
        metadataFile.close();
    }
    spdlog::info("Behavior image saver thread stopped");
}