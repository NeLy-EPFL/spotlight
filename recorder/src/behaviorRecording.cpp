#include "behaviorRecording.hpp"

namespace
{
    std::set<std::string> intializedBaseDirectories;

    std::filesystem::path prepareBehaviorImageDir(std::string baseDirectory)
    {
        std::filesystem::path behaviorSaveDir;
        {
            std::lock_guard<std::mutex> lock(isIOInitializing);
            if (intializedBaseDirectories.find(baseDirectory) ==
                intializedBaseDirectories.end())
            {
                behaviorSaveDir = prepareOutputFolder(
                    std::filesystem::path(baseDirectory) / "behavior_images",
                    true);
                intializedBaseDirectories.insert(baseDirectory);
            }
        }
        return behaviorSaveDir;
    }
}

void behaviorImageAcquierer()
{
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

    bool isFristFrameRecorded = true;

    while (!toQuit.load())
    {
        // Acquire image data
        // // Benchmark here shows that the waitForOneFrame() function takes
        // // on average (triggeringCyclePeriod - 150) us to complete. So we
        // // have plenty of margin and can theoretically record at
        // // 1,000,000 / 200-ish = 5,000 fps.
        // // uint64_t startTime = getCurrentTimeMicroseconds();
        FrameData frameData = behaviorCamera->waitForOneFrame();
        // uint64_t waitTime = getCurrentTimeMicroseconds() - startTime;
        // spdlog::info("Behavior camera waited {} us", waitTime);

        // Update latest frame for live display
        {
            std::lock_guard<std::mutex> lock(latestFrameMutex);
            latestFrameData = frameData;
        }

        if (isRecording.load())
        {
            if (isFristFrameRecorded)
            {
                // Reset these to 0 in preparation for the next recording session
                frameDataBufferIndex = 0;
                currentFrameId = 0;
                isFristFrameRecorded = false; // toggle off
            }

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
    std::thread::id myThreadId = std::this_thread::get_id();
    std::stringstream ss;
    ss << myThreadId;
    std::string threadIdString = ss.str();

    // Define OpenCV JPEG saving parameters
    std::vector<int> compressionParams;
    compressionParams.push_back(cv::IMWRITE_JPEG_QUALITY);
    compressionParams.push_back(100); // Maximum quality, minimal compression
    compressionParams.push_back(cv::IMWRITE_JPEG_CHROMA_QUALITY);
    compressionParams.push_back(100); // Maximum quality, minimal compression
    // Unclear why OpenCV is built without this option. It should because I'm
    // on OpenCV 4.8 with libjpeg ver 80. This option directly specifies the
    // type of chroma subsampling used. This is about resolution of the chroma
    // channels rather than compression quality. Common values: 444 = No chroma
    // subsampling (full resolution for chroma channels)
    // compressionParams.push_back(cv::IMWRITE_JPEG_SAMPLING_FACTOR);
    // compressionParams.push_back(444); // Disable chroma subsampling (4:4:4)

    int iterCount = 0;
    std::filesystem::path behaviorSaveDir;

    while (!toQuit.load())
    {
        GroupOfThreeFrames frameGroup;
        int queueLength;
        {
            std::unique_lock<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueueCondVar.wait(
                lock, []
                { return !behaviorImageQueue.empty() || toQuit.load(); });

            if (toQuit.load())
            {
                spdlog::info(
                    "Behavior image saver thread is breaking out of loop.");
                break;
            }

            queueLength = behaviorImageQueue.size();
            frameGroup = behaviorImageQueue.front();
            behaviorImageQueue.pop();
        }

        uint64_t startTime = getCurrentTimeMicroseconds();

        bool couldBeFirstFrame =
            frameGroup.frame0.frameId < 3 * NUM_BEHAVIOR_IMAGE_SAVING_THREADS;
        if (couldBeFirstFrame) {
            behaviorSaveDir = prepareBehaviorImageDir(saveDirectory);
        }

        std::string filenameStem =
            "behavior_frame_" +
            fmt::format("{:09}", frameGroup.frame0.frameId);

        // Save three frames as a single pseudo-RGB image
        std::string filename = behaviorSaveDir / (filenameStem + ".jpg");
        cv::Mat image = makePseudoRGBImageFromThreeFrames(frameGroup);
        cv::imwrite(filename, image, compressionParams);

        // Save metadata
        std::string metadataFilename =
            behaviorSaveDir / (filenameStem + ".csv");
        std::ofstream metadataFile(metadataFilename);
        metadataFile << makeMetadataStringFromThreeFrames(frameGroup);
        metadataFile.close();

        uint64_t walltime = getCurrentTimeMicroseconds() - startTime;
        bool shouldLogPerformance =
            iterCount % LOG_BEHAVIOR_CAMERA_SAVE_PERFORMANCE_INTERVAL == 0;
        if (shouldLogPerformance)
        {
            spdlog::info(
                "Behavior image saver thread (thread ID {}) reporting: "
                "{} frames in queue; "
                "it took {} us to save a group of three frames",
                threadIdString, queueLength, walltime);
        }
        iterCount++;
    }
    spdlog::info("Behavior image saver thread stopped");
}

void stopBehaviorImageSaver()
{
    if (!toQuit.load())
    {
        spdlog::critical(
            "stopBehaviorImageSaver() called but toQuit is "
            "not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stopBehaviorImageSaver() called but toQuit is "
            "not set to true. This shouldn't happen.");
    }
    else
    {
        behaviorImageQueueCondVar.notify_all();
    }
}