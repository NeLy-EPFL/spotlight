#include "recordingController.hpp"

namespace
{
    BehaviorCamera *behaviorCamera = nullptr;
}

void behaviorImageAcquierer()
{
    std::signal(SIGINT, [](int)
                { quitProgram(); });

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

    while (!toQuit->load())
    {
        GroupOfThreeFrames frameGroup;
        int queueLength;
        {
            std::unique_lock<std::mutex> lock(behaviorImageQueueMutex);
            behaviorImageQueueCondVar.wait(
                lock, []
                { return !behaviorImageQueue.empty(); });
            queueLength = behaviorImageQueue.size();
            frameGroup = behaviorImageQueue.front();
            behaviorImageQueue.pop();
        }

        if (frameGroup.frame0.noMoreData)
        {
            spdlog::info("Behavior image saver received stopper; "
                         "no more frame will come.");
            break;
        }

        uint64_t startTime = getCurrentTimeMicroseconds();
        std::string filenameStem =
            saveDirectory +
            "/behavior_" +
            fmt::format("{:09}", frameGroup.frame0.frameId);

        // Save three frames as a single pseudo-RGB image
        std::string filename = filenameStem + ".jpg";
        cv::Mat image = makePseudoRGBImageFromThreeFrames(frameGroup);
        cv::imwrite(filename, image, compressionParams);

        // Save metadata
        std::string metadataFilename = filenameStem + ".txt";
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

void quitProgram()
/**
 * Quit gracefully by explicitly stopping acquisition on the behavior
 * camera* and telling saver threads that the work is done.
 *
 * * Without stopping acquiisition explicitly, the frame grabber will
 * think the device is still busy the next time we run the program.
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