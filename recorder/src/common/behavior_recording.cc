#include "recorder/common/behavior_recording.h"

#include "recorder/common/saver_perf_tracker.h"

void behaviorImageAcquirer(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop) {
    spdlog::info("Behavior image acquirer thread started");
    BehaviorCameraROI cameraROI = getBehaviorBehaviorCameraROI(recorderConfig);

    std::string frameGrabberTriggerLine =
        recorderConfig.getParameter<std::string>(
            "behavior_camera", "frame_grabber_trigger_line");
    behaviorRecordingState->behaviorCamera = std::make_shared<BehaviorCamera>(
        cameraROI.imageWidth,
        cameraROI.imageHeight,
        cameraROI.xOffset,
        cameraROI.yOffset,
        frameGrabberTriggerLine);

    spdlog::info("Behavior camera configured");

    behaviorRecordingState->behaviorCamera->start();
    spdlog::info("Behavior camera started");

    FrameData frameDataBuffer[3];
    size_t frameDataBufferIndex = 0;
    long int currentFrameId = 0;

    bool wasRecording = false;
    // Set once this thread has acquired exactly the programmed number of frames
    // and stopped recording on its own, so subsequent frames are discarded
    // until the GUI tears the recording down.
    bool reachedProgrammedStop = false;

    // Flush a partial group of one or two buffered frames as a single
    // pseudo-BGR image (the missing channel(s) are saved black and are not
    // logged in the CSV metadata; see makePseudoBGRImageFromThreeFrames and
    // makeMetadataStringFromThreeFrames). Resets the buffer index.
    auto flushPartialGroup = [&]() {
        GroupOfThreeFrames partialGroup;
        partialGroup.frame0 = frameDataBuffer[0];
        if (frameDataBufferIndex > 1) {
            partialGroup.frame1 = frameDataBuffer[1];
        }
        partialGroup.numValidFrames = static_cast<int>(frameDataBufferIndex);
        {
            std::lock_guard<std::mutex> lock(
                behaviorRecordingState->behaviorImageQueueMutex);
            behaviorRecordingState->behaviorImageQueue.push(partialGroup);
        }
        behaviorRecordingState->behaviorImageQueueCondVar.notify_one();
        frameDataBufferIndex = 0;
    };

    while (!programState->toQuit.load()) {
        // Acquire image data
        // // Benchmark here shows that the waitForOneFrame() function takes
        // // on average (triggeringCyclePeriod - 150) us to complete. So we
        // // have plenty of margin and can theoretically record at
        // // 1,000,000 / 200-ish = 5,000 fps.
        // // uint64_t startTime = getCurrentTimeMicroseconds();
        // spdlog::debug(
        //     "Behavior image acquirer thread waiting for one frame");
        FrameData frameData =
            behaviorRecordingState->behaviorCamera->waitForOneFrame();
        // spdlog::debug(
        //     "Behavior image acquirer thread received one frame");
        // uint64_t waitTime = getCurrentTimeMicroseconds() - startTime;
        // spdlog::info("Behavior camera waited {} us", waitTime);

        // Update latest frame for live display
        behaviorRecordingState->latestFrameHolder->setLatestFrameData(
            frameData);

        bool isRecording = programState->isRecording.load();

        if (isRecording && !wasRecording) {
            // Start of a new recording session: reset the counters.
            frameDataBufferIndex = 0;
            currentFrameId = 0;
            reachedProgrammedStop = false;
        }

        int numFramesExpected =
            programmedRecordingStop->numBehaviorFramesExpected;

        if (isRecording && !reachedProgrammedStop) {
            frameData.frameId = currentFrameId;

            frameDataBuffer[frameDataBufferIndex++] = frameData;

            if (frameDataBufferIndex == 3) {
                // Add a full group of three frames to the queue.
                GroupOfThreeFrames groupOfThreeFrames = {
                    frameDataBuffer[0],
                    frameDataBuffer[1],
                    frameDataBuffer[2],
                    3};
                {
                    std::lock_guard<std::mutex> lock(
                        behaviorRecordingState->behaviorImageQueueMutex);
                    behaviorRecordingState->behaviorImageQueue.push(
                        groupOfThreeFrames);
                }
                behaviorRecordingState->behaviorImageQueueCondVar.notify_one();

                frameDataBufferIndex = 0;
            }

            // Stop exactly on the programmed frame count. Once the last expected
            // frame has been acquired, flush any partial group and stop
            // recording right here, rather than waiting for the GUI to tear the
            // recording down (which would overrun by however many frames arrive
            // during the GUI's poll latency). The GUI is notified via
            // programmedStopReached so it can finalize the UI and revert the
            // cameras to streaming.
            if (numFramesExpected >= 0 &&
                currentFrameId == numFramesExpected - 1) {
                if (frameDataBufferIndex > 0) {
                    flushPartialGroup();
                }
                reachedProgrammedStop = true;
                programmedRecordingStop->programmedStopReached.store(true);
                spdlog::info(
                    "Programmed stop reached after {} behavior frames. Behavior "
                    "acquisition thread stopped recording and is telling the "
                    "GUI to finalize.",
                    numFramesExpected);
            }

            currentFrameId++;
        } else {
            if (wasRecording && !reachedProgrammedStop &&
                frameDataBufferIndex > 0) {
                // A user-initiated stop landed on a partial group of one or two
                // frames; flush it. (A programmed stop has already flushed its
                // own partial group above.)
                flushPartialGroup();
            }

            if (!isRecording) {
                // Not recording: any newly arrived frame is discarded (it is
                // only used for the live preview above). Reset the counters for
                // the next recording session.
                frameDataBufferIndex = 0;
                currentFrameId = 0;
                reachedProgrammedStop = false;
            }
        }

        wasRecording = isRecording;
    }

    // Stop behavior camera acquisition
    spdlog::info("Stopping acquisition on behavior camera");
    behaviorRecordingState->behaviorCamera->stop();
    spdlog::info("Behavior camera acquisition stopped. "
                 "Behavior image acquirer thread reached its end");
}

void behaviorImageSaver(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState) {
    std::thread::id myThreadId = std::this_thread::get_id();
    std::stringstream ss;
    ss << myThreadId;
    std::string threadIdString = ss.str();
    spdlog::info(
        "Behavior image saver thread started (thread ID {})", threadIdString);

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

    int queueLength = -1;

    SaverPerfTracker perfTracker(
        "Behavior image saver thread", "group of three frames", threadIdString);

    while (!programState->toQuit.load()) {
        GroupOfThreeFrames frameGroup;
        {
            std::unique_lock<std::mutex> lock(
                behaviorRecordingState->behaviorImageQueueMutex);
            behaviorRecordingState->behaviorImageQueueCondVar.wait(
                lock, [&behaviorRecordingState, programState] {
                    return !behaviorRecordingState->behaviorImageQueue
                                .empty() ||
                           programState->toQuit.load();
                });

            if (programState->toQuit.load()) {
                spdlog::info(
                    "Behavior image saver thread is breaking out of loop.");
                break;
            }

            queueLength = behaviorRecordingState->behaviorImageQueue.size();
            frameGroup = behaviorRecordingState->behaviorImageQueue.front();
            behaviorRecordingState->behaviorImageQueue.pop();
        }

        perfTracker.updateRecordingState(programState->isRecording.load());

        uint64_t startTime = getCurrentTimeMicroseconds();

        std::string filenameStem =
            "behavior_frame_" + fmt::format("{:09}", frameGroup.frame0.frameId);
        std::filesystem::path behaviorSaveDir =
            std::filesystem::path(saveDirectory->getDirectory()) /
            "behavior_images";

        // Save three frames as a single pseudo-BGR image
        std::string filename = behaviorSaveDir / (filenameStem + ".jpg");
        cv::Mat image = makePseudoBGRImageFromThreeFrames(frameGroup);
        cv::imwrite(filename, image, compressionParams);

        // Save metadata
        std::string metadataFilename =
            behaviorSaveDir / (filenameStem + ".csv");
        std::ofstream metadataFile(metadataFilename);
        metadataFile << makeMetadataStringFromThreeFrames(frameGroup);
        metadataFile.close();

        perfTracker.recordSave(
            getCurrentTimeMicroseconds() - startTime, queueLength);
    }
    spdlog::info("Behavior image saver thread stopped");
}

void stopBehaviorImageSaver(
    std::shared_ptr<BehaviorRecordingState> behaviorRecordingState,
    std::shared_ptr<ProgramState> programState) {
    if (!programState->toQuit.load()) {
        spdlog::critical("stopBehaviorImageSaver() called but toQuit is "
                         "not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stopBehaviorImageSaver() called but toQuit is "
            "not set to true. This shouldn't happen.");
    } else {
        behaviorRecordingState->behaviorImageQueueCondVar.notify_all();
    }
}

BehaviorCameraROI
getBehaviorBehaviorCameraROI(const RecorderConfig &recorderConfig) {
    int imageWidth = roundToNearestValidBehaviorCamDimension(
        recorderConfig.getParameter<int>("behavior_camera", "roi_width"));
    int imageHeight = roundToNearestValidBehaviorCamDimension(
        recorderConfig.getParameter<int>("behavior_camera", "roi_height"));
    int fullFrameWidth =
        recorderConfig.getParameter<int>("behavior_camera", "full_frame_width");
    int fullFrameHeight = recorderConfig.getParameter<int>(
        "behavior_camera", "full_frame_height");

    if (imageWidth < 0 || imageHeight < 0 || fullFrameWidth < 0 ||
        fullFrameHeight < 0 || imageWidth > fullFrameWidth ||
        imageHeight > fullFrameHeight) {
        std::string errorMessage = fmt::format(
            "Invalid camera ROI or full frame size: "
            "imageWidth = {}, imageHeight = {}, "
            "fullFrameWidth = {}, fullFrameHeight = {}",
            imageWidth,
            imageHeight,
            fullFrameWidth,
            fullFrameHeight);
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }

    auto [xOffset, yOffset] = getCenteredOffsets(
        imageWidth, imageHeight, fullFrameWidth, fullFrameHeight);

    BehaviorCameraROI cameraROI = {
        (unsigned int)imageWidth,
        (unsigned int)imageHeight,
        (unsigned int)xOffset,
        (unsigned int)yOffset};
    return cameraROI;
}