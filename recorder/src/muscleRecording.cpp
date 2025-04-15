#include "muscleRecording.hpp"

void muscleImageAcquierer(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    const RecorderConfig &recorderConfig,
    std::string profileDir,
    spdlog::level::level_enum logLevel,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop)
{
    spdlog::info("Muscle image acquirer thread started");

    // Create muscle camera
    muscleRecordingState->muscleCamera =
        std::make_shared<MuscleCamera>(imageWidth,
                                       imageHeight,
                                       xOffset,
                                       yOffset,
                                       recorderConfig,
                                       profileDir,
                                       logLevel);

    spdlog::info("Muscle camera configured. Entering frame grabbing loop...");
    long int currentFrameId = 0;
    bool notExpectingMoreFrames = false;

    while (!programState->toQuit.load())
    {
        FrameData frameData =
            muscleRecordingState->muscleCamera->waitForOneFrame();
        if (frameData.image.empty())
        {
            spdlog::error("muscleImageAcquierer thread got an empty image");
        }
        muscleRecordingState
            ->latestBehaviorFrameHolder
            ->setLatestFrameData(frameData);
        if (muscleRecordingState->latestBehaviorFrameHolder->getLatestFrameData().image.empty())
        {
            spdlog::error("last empty image");
        }

        if (programState->isRecording.load())
        {
            frameData.frameId = currentFrameId++;
            {
                std::lock_guard<std::mutex> lock(
                    muscleRecordingState->muscleImageQueueMutex);
                muscleRecordingState->muscleImageQueue.push(frameData);
            }
            muscleRecordingState->muscleImageQueueCondVar.notify_one();

            if (notExpectingMoreFrames)
            {
                spdlog::error(
                    "Muscle camera is not expecting more frames because a "
                    "programmed stop was reached, but it got one anyway!");
            }

            // Whether we've reached a programmed stop
            int numFramesExpected =
                programmedRecordingStop->numMuscleFramesExpected;
            if (currentFrameId == numFramesExpected &&
                numFramesExpected >= 0)
            {
                // Placeholder - nothing to do here actually because the
                // Arduino will stop triggering the muscle camera by itself
                // Don't toggle programmedRecordingStop->hasEndedFlagForGUI (the
                // behavior acquirer thread will do it)
                spdlog::info("Muscle camera reached programmed stop.");
                notExpectingMoreFrames = true;
            }
        }
        else
        {
            // If we're not recording, we need to reset the frame ID
            // counter so that the next recording session starts at 0
            currentFrameId = 0;
            notExpectingMoreFrames = false;
        }
    }
}

void muscleImageSaver(
    const RecorderConfig &recorderConfig,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<SaveDirectory> saveDirectory,
    std::shared_ptr<ProgramState> programState,
    int tiffCompressionMethod)
{
    std::thread::id myThreadId = std::this_thread::get_id();
    std::stringstream ss;
    ss << myThreadId;
    std::string threadIdString = ss.str();
    spdlog::info("Muscle image saver thread started (thread ID {})",
                 threadIdString);

    std::vector<int> compressionParams;
    compressionParams.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    compressionParams.push_back(tiffCompressionMethod);

    int frameCount = 0;
    int queueLength = -1;
    uint64_t startTime = 0;
    uint64_t walltime = 0;
    FrameData frameData;

    int performanceLoggingInterval = recorderConfig.getParameter<int>(
        "muscle_camera", "saving_performance_logging_interval");

    while (!programState->toQuit.load())
    {
        {
            std::unique_lock<std::mutex> lock(
                muscleRecordingState->muscleImageQueueMutex);
            muscleRecordingState->muscleImageQueueCondVar.wait(
                lock, [&muscleRecordingState, programState]
                { return !muscleRecordingState->muscleImageQueue.empty() ||
                         programState->toQuit.load(); });

            if (programState->toQuit.load())
            {
                spdlog::info(
                    "Behavior image saver thread is breaking out of loop.");
                break;
            }

            queueLength = muscleRecordingState->muscleImageQueue.size();
            frameData = muscleRecordingState->muscleImageQueue.front();
            muscleRecordingState->muscleImageQueue.pop();
        }

        startTime = getCurrentTimeMicroseconds();
        std::string filenameStem =
            "behavior_frame_" + fmt::format("{:09}", frameData.frameId);
        std::filesystem::path muscleSaveDir =
            std::filesystem::path(saveDirectory->getDirectory()) /
            "muscle_images";

        std::string filename = muscleSaveDir / (filenameStem + ".tif");

        try
        {
            cv::imwrite(filename, frameData.image, compressionParams);
        }
        catch (const cv::Exception &ex)
        {
            spdlog::error("Exception saving image: {}", ex.what());
        }

        walltime = getCurrentTimeMicroseconds() - startTime;
        if (frameCount % performanceLoggingInterval)
        {
            spdlog::info(
                "Muscle image saver thread (thread ID {}) reporting: "
                "{} frames in queue; "
                "it took {} us to save a single frame",
                threadIdString, queueLength, walltime);
        }
    }
}

void stopMuscleImageSaver(
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState)
{
    if (!programState->toQuit.load())
    {
        spdlog::critical(
            "stopMuscleImageSaver() called but toQuit is "
            "not set to true. This shouldn't happen.");
        throw std::runtime_error(
            "stopMuscleImageSaver() called but toQuit is "
            "not set to true. This shouldn't happen.");
    }
    else
    {
        muscleRecordingState->muscleImageQueueCondVar.notify_all();
    }
}
