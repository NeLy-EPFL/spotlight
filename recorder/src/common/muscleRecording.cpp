#include "muscleRecording.hpp"

MuscleCameraROI::MuscleCameraROI(int x0, int x1, int y0, int y1)
    : x0(x0), x1(x1), y0(y0), y1(y1), xOffset(x0 - 1), yOffset(y0 - 1),
      imageWidth(x1 - x0 + 1), imageHeight(y1 - y0 + 1) {}

bool MuscleCameraROI::isWithinBound(int fullWidth, int fullHeight) {
    return (
        x0 > 0 && x1 <= fullWidth && y0 > 0 && y1 <= fullHeight && x0 < x1 &&
        y0 < y1);
}

int MuscleCameraROI::toFile(std::filesystem::path path) {
    YAML::Node node;
    node["x0"] = x0;
    node["x1"] = x1;
    node["y0"] = y0;
    node["y1"] = y1;
    node["xOffset"] = xOffset;
    node["yOffset"] = yOffset;
    node["imageWidth"] = imageWidth;
    node["imageHeight"] = imageHeight;

    std::ofstream fout(path);
    if (!fout) {
        spdlog::error("Failed to open file: {}", path.string());
        return 1;
    }

    fout << node;
    fout.close();
    return 0;
}

std::tuple<int, int> MuscleCameraROI::getCenterXY() {
    return std::make_tuple((x0 + x1) / 2, (y0 + y1) / 2);
}

MuscleCameraROI getMuscleCameraROI(std::filesystem::path roiFilePath) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(roiFilePath.string());
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(fmt::format(
            "Failed to load muscle camera ROI from {}: {}",
            roiFilePath.string(),
            e.what()));
    }

    auto readInt = [&](const char *key) {
        if (!node[key]) {
            throw std::runtime_error(fmt::format(
                "Muscle camera ROI file {} is missing key '{}'",
                roiFilePath.string(),
                key));
        }
        try {
            return node[key].as<int>();
        } catch (const YAML::Exception &e) {
            throw std::runtime_error(fmt::format(
                "Muscle camera ROI key '{}' in {} is not an int: {}",
                key,
                roiFilePath.string(),
                e.what()));
        }
    };

    int x0 = readInt("x0");
    int x1 = readInt("x1");
    int y0 = readInt("y0");
    int y1 = readInt("y1");
    int imageWidth = readInt("imageWidth");
    int imageHeight = readInt("imageHeight");
    MuscleCameraROI roi(x0, x1, y0, y1);

    spdlog::info(
        "Muscle camera ROI loaded from file: x0={}, x1={}, y0={}, y1={}; "
        "imageWidth={}, imageHeight={}",
        x0,
        x1,
        y0,
        y1,
        imageWidth,
        imageHeight);
    return roi;
}

void muscleImageAcquirer(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    const RecorderConfig &recorderConfig,
    std::string profileDir,
    spdlog::level::level_enum logLevel,
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState,
    std::shared_ptr<ProgrammedStop> programmedRecordingStop) {
    spdlog::info("Muscle image acquirer thread started");

    // Create muscle camera
    double rollingShutterLineTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensorReadoutTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    muscleRecordingState->muscleCamera = std::make_shared<MuscleCamera>(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        rollingShutterLineTimeUs,
        sensorReadoutTimeUs,
        recorderConfig,
        profileDir,
        logLevel);

    spdlog::info("Muscle camera configured. Entering frame grabbing loop...");
    long int currentFrameId = 0;
    bool notExpectingMoreFrames = false;

    while (!programState->toQuit.load()) {
        FrameData frameData =
            muscleRecordingState->muscleCamera->waitForOneFrame();
        if (frameData.image.empty()) {
            spdlog::error("muscleImageAcquirer thread got an empty image");
        }
        muscleRecordingState->latestFrameHolder->setLatestFrameData(frameData);

        if (programState->isRecording.load()) {
            frameData.frameId = currentFrameId++;
            {
                std::lock_guard<std::mutex> lock(
                    muscleRecordingState->muscleImageQueueMutex);
                muscleRecordingState->muscleImageQueue.push(frameData);
            }
            muscleRecordingState->muscleImageQueueCondVar.notify_one();

            if (notExpectingMoreFrames) {
                spdlog::error(
                    "Muscle camera is not expecting more frames because a "
                    "programmed stop was reached, but it got one anyway!");
            }

            // Whether we've reached a programmed stop
            int numFramesExpected =
                programmedRecordingStop->numMuscleFramesExpected;
            if (currentFrameId == numFramesExpected && numFramesExpected >= 0) {
                // Placeholder - nothing to do here actually because the
                // Arduino will stop triggering the muscle camera by itself
                // Don't toggle programmedRecordingStop->hasEndedFlagForGUI (the
                // behavior acquirer thread will do it)
                spdlog::info("Muscle camera reached programmed stop.");
                notExpectingMoreFrames = true;
            }
        } else {
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
    int tiffCompressionMethod) {
    std::thread::id myThreadId = std::this_thread::get_id();
    std::stringstream ss;
    ss << myThreadId;
    std::string threadIdString = ss.str();
    spdlog::info(
        "Muscle image saver thread started (thread ID {})", threadIdString);

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

    while (!programState->toQuit.load()) {
        {
            std::unique_lock<std::mutex> lock(
                muscleRecordingState->muscleImageQueueMutex);
            muscleRecordingState->muscleImageQueueCondVar.wait(
                lock, [&muscleRecordingState, programState] {
                    return !muscleRecordingState->muscleImageQueue.empty() ||
                           programState->toQuit.load();
                });

            if (programState->toQuit.load()) {
                spdlog::info(
                    "Muscle image saver thread is breaking out of loop.");
                break;
            }

            queueLength = muscleRecordingState->muscleImageQueue.size();
            frameData = muscleRecordingState->muscleImageQueue.front();
            muscleRecordingState->muscleImageQueue.pop();
        }
        startTime = getCurrentTimeMicroseconds();
        std::string filenameStem =
            "muscle_frame_" + fmt::format("{:09}", frameData.frameId);
        std::filesystem::path muscleSaveDir =
            std::filesystem::path(saveDirectory->getDirectory()) /
            "muscle_images";

        // Reorient image (rotate it so it's consistent with behavior image)
        cv::Mat reorientedImage;
        reorientMuscleImage(frameData.image, reorientedImage);

        // Save image
        std::string imagePath = muscleSaveDir / (filenameStem + ".tif");
        try {
            cv::imwrite(imagePath, reorientedImage, compressionParams);
        } catch (const cv::Exception &ex) {
            spdlog::error("Exception saving image: {}", ex.what());
        }

        // Save metadata
        std::string metadataPath = muscleSaveDir / (filenameStem + ".csv");
        std::ofstream metadataFile(metadataPath);
        if (!metadataFile.is_open()) {
            spdlog::error(
                "Failed to open metadata file: {}", metadataPath.c_str());
        } else {
            metadataFile << "frame_id,acquired_time_us,received_time_us\n";
            metadataFile << frameData.frameId << ","
                         << frameData.acquisitionTime << ","
                         << frameData.receivedTime << "\n";
            metadataFile.close();
        }

        // Profiling and logging to monitor performance
        walltime = getCurrentTimeMicroseconds() - startTime;
        if (frameCount % performanceLoggingInterval) {
            spdlog::info(
                "Muscle image saver thread (thread ID {}) reporting: "
                "{} frames in queue; "
                "it took {} us to save a single frame",
                threadIdString,
                queueLength,
                walltime);
        }
    }
}

void stopMuscleImageSaver(
    std::shared_ptr<MuscleRecordingState> muscleRecordingState,
    std::shared_ptr<ProgramState> programState) {
    if (!programState->toQuit.load()) {
        spdlog::critical("stopMuscleImageSaver() called but toQuit is "
                         "not set to true. This shouldn't happen.");
        throw std::runtime_error("stopMuscleImageSaver() called but toQuit is "
                                 "not set to true. This shouldn't happen.");
    } else {
        muscleRecordingState->muscleImageQueueCondVar.notify_all();
    }
}
