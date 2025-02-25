#include "recordingController.hpp"

namespace
{
    BehaviorCamera *behaviorCamera = nullptr;

    std::mutex motionStageRequestMutex;
    std::condition_variable motionStageRequestCondVar;
    std::mutex motionStageResponseMutex;
    std::condition_variable motionStageResponseCondVar;
    std::atomic<bool> newRequestForMotionstage;
    std::atomic<bool> newPositionFromMotionStage;
    MotionStageRequest latestMotionStageRequest;
    MotionStagePosition latestMotionStagePosition;
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

void motionControlRequestHandler()
{
    auto [numMicrosecsAllowedGet, numMicrosecsAllowedSet] =
        calculateMaxMotionStageRequestHandlingTime();

    MotionControl motionControl;
    motionControlHandlerReady.store(true);

    while (!toQuit->load()) // TODO: remove this?
    {
        MotionStageRequest myRequest;
        {
            std::unique_lock<std::mutex> lock(motionStageRequestMutex);
            motionStageRequestCondVar.wait(
                lock, []
                { return newRequestForMotionstage.load(); });
            myRequest = latestMotionStageRequest;
            newRequestForMotionstage = false;
        }

        uint64_t startTime = getCurrentTimeMicroseconds();

        if (myRequest.requestType == SET_TARGET_POSITION)
        {
            motionControl.moveAbsolute(
                X_AXIS, myRequest.position.xPosAbsoluteMm, false);
            motionControl.moveAbsolute(
                Y_AXIS, myRequest.position.yPosAbsoluteMm, false);
        }
        else if (myRequest.requestType == GET_CURRENT_POSITION)
        {
            MotionStagePosition currentPosition;
            currentPosition.xPosAbsoluteMm =
                motionControl.getPosition(X_AXIS);
            currentPosition.yPosAbsoluteMm =
                motionControl.getPosition(Y_AXIS);
            {
                std::lock_guard<std::mutex> lock(motionStageResponseMutex);
                latestMotionStagePosition = currentPosition;
                newPositionFromMotionStage = true;
            }
            motionStageResponseCondVar.notify_one();
        }
        else if (myRequest.requestType == HOME)
        {
            motionControl.home(X_AXIS);
            motionControl.home(Y_AXIS);
            motionControl.waitUntilIdle(X_AXIS);
            motionControl.waitUntilIdle(Y_AXIS);
        }
        else if (myRequest.requestType == QUIT)
        {
            spdlog::info("Motion control request handler thread received "
                         "QUIT request. Breaking out of loop.");
            break;
        }
        else
        {
            spdlog::error("Unknown motion stage request type {}",
                          myRequest.requestType);
        }

        uint64_t walltime = getCurrentTimeMicroseconds() - startTime;

        int walltimeLimit;
        switch (myRequest.requestType)
        {
        case GET_CURRENT_POSITION:
            walltimeLimit = numMicrosecsAllowedGet;
            break;
        case SET_TARGET_POSITION:
            walltimeLimit = numMicrosecsAllowedSet;
            break;
        default:
            walltimeLimit = INT_MAX;
            break;
        }

        if (walltime > walltimeLimit)
        {
            spdlog::warn(
                "Motion control request handler thread taking more than "
                "allotted time to handle request type {}; walltime {} us "
                "(allowed {} us). Note that the allowed time is only the "
                "AVERAGE amount of time that this thread can spend on each "
                "operation. It's harmless if this message appears only "
                "sporadically.",
                myRequest.requestType, walltime, walltimeLimit);
        }
    }

    spdlog::info("Motion control request handler thread stopped");
}

MotionStagePosition getCurrentMotionStagePosition()
{
    MotionStageRequest request;
    request.requestType = GET_CURRENT_POSITION;
    {
        std::lock_guard<std::mutex> lock(motionStageRequestMutex);
        latestMotionStageRequest = request;
        newRequestForMotionstage = true;
    }
    motionStageRequestCondVar.notify_one();

    {
        std::unique_lock<std::mutex> lock(motionStageResponseMutex);
        motionStageResponseCondVar.wait(
            lock, []
            { return newPositionFromMotionStage.load() || toQuit->load(); });
        newPositionFromMotionStage = false;
    }

    return latestMotionStagePosition;
}

void setTargetMotionStagePosition(MotionStagePosition targetPosition)
{
    MotionStageRequest request;
    request.requestType = SET_TARGET_POSITION;
    request.position = targetPosition;
    {
        std::lock_guard<std::mutex> lock(motionStageRequestMutex);
        latestMotionStageRequest = request;
        newRequestForMotionstage = true;
    }
    motionStageRequestCondVar.notify_one();
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

    toQuit->store(true);

    // Stop behavior camera acquisition
    if (behaviorCamera)
    {
        spdlog::info("Stopping acquisition on behavior camera");
        behaviorCamera->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop "
                 "by sending a QUIT request to it");
    MotionStageRequest stopper;
    stopper.requestType = QUIT;
    {
        std::lock_guard<std::mutex> lock(motionStageRequestMutex);
        latestMotionStageRequest = stopper;
        newRequestForMotionstage.store(true);
    }
    motionStageRequestCondVar.notify_one();

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