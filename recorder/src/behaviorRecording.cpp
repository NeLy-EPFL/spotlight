#include "behaviorRecording.hpp"

void behaviorImageAcquirer(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<LatestFrame> latestBehaviorFrameHolder,
    std::shared_ptr<ProgramState> programState)
{
    spdlog::info("Behavior image acquirer thread started");
    unsigned int imageWidth = roundToMultiplesOf64(
        recorderConfig.getParameter<int>("behavior_camera", "roi_width"));
    unsigned int imageHeight = roundToMultiplesOf64(
        recorderConfig.getParameter<int>("behavior_camera", "roi_height"));
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    std::string frameGrabberTriggerLine =
        recorderConfig.getParameter<std::string>("behavior_camera",
                                                 "frame_grabber_trigger_line");
    behaviorRecordingState->behaviorCamera =
        std::make_shared<BehaviorCamera>(imageWidth,
                                         imageHeight,
                                         xOffset,
                                         yOffset,
                                         frameGrabberTriggerLine);

    spdlog::info("Behavior camera configured");

    behaviorRecordingState->behaviorCamera->start();
    spdlog::info("Behavior camera started");

    FrameData frameDataBuffer[3];
    size_t frameDataBufferIndex = 0;
    long int currentFrameId = 0;

    bool isFristFrameRecorded = true;

    while (!programState->toQuit.load())
    {
        // Acquire image data
        // // Benchmark here shows that the waitForOneFrame() function takes
        // // on average (triggeringCyclePeriod - 150) us to complete. So we
        // // have plenty of margin and can theoretically record at
        // // 1,000,000 / 200-ish = 5,000 fps.
        // // uint64_t startTime = getCurrentTimeMicroseconds();
        FrameData frameData =
            behaviorRecordingState->behaviorCamera->waitForOneFrame();
        // uint64_t waitTime = getCurrentTimeMicroseconds() - startTime;
        // spdlog::info("Behavior camera waited {} us", waitTime);

        // Update latest frame for live display
        {
            latestBehaviorFrameHolder->setLatestFrameData(frameData);
        }

        if (programState->isRecording.load())
        {
            if (isFristFrameRecorded)
            {
                // Reset these to 0 in preparation for the next recording
                // session
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
                    std::lock_guard<std::mutex> lock(
                        behaviorRecordingState->behaviorImageQueueMutex);
                    behaviorRecordingState->behaviorImageQueue.push(
                        groupOfThreeFrames);
                }
                behaviorRecordingState->behaviorImageQueueCondVar.notify_one();

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
void behaviorImageSaver(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState)
{
    std::thread::id myThreadId = std::this_thread::get_id();
    std::stringstream ss;
    ss << myThreadId;
    std::string threadIdString = ss.str();
    spdlog::info("Behavior image saver thread started (thread ID {})",
                 threadIdString);

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

    std::set<std::string> initializedSaveDirectories; // root save directories

    int performanceLoggingInterval = recorderConfig.getParameter<int>(
        "behavior_camera", "saving_performance_logging_interval");

    while (!programState->toQuit.load())
    {
        GroupOfThreeFrames frameGroup;
        int queueLength;
        {
            std::unique_lock<std::mutex> lock(
                behaviorRecordingState->behaviorImageQueueMutex);
            behaviorRecordingState->behaviorImageQueueCondVar.wait(
                lock, [&behaviorRecordingState, programState]
                { return !behaviorRecordingState->behaviorImageQueue.empty() ||
                         programState->toQuit.load(); });

            if (programState->toQuit.load())
            {
                spdlog::info(
                    "Behavior image saver thread is breaking out of loop.");
                break;
            }

            queueLength = behaviorRecordingState->behaviorImageQueue.size();
            frameGroup = behaviorRecordingState->behaviorImageQueue.front();
            behaviorRecordingState->behaviorImageQueue.pop();
        }

        uint64_t startTime = getCurrentTimeMicroseconds();

        std::string filenameStem =
            "behavior_frame_" +
            fmt::format("{:09}", frameGroup.frame0.frameId);
        std::filesystem::path behaviorSaveDir =
            std::filesystem::path(saveDirectory->getDirectory()) /
            "behavior_images";

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
        if (iterCount % performanceLoggingInterval == 0)
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

void stopBehaviorImageSaver(
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<ProgramState> programState)
{
    if (!programState->toQuit.load())
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
        behaviorRecordingState->behaviorImageQueueCondVar.notify_all();
    }
}