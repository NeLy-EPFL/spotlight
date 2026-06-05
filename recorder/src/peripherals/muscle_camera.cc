#include "recorder/peripherals/muscle_camera.h"

#include <filesystem>

namespace {
std::string logLevelToStr(spdlog::level::level_enum logLevel) {
    switch (logLevel) {
    case spdlog::level::trace:
        return "trace";
    case spdlog::level::debug:
        return "debug";
    case spdlog::level::info:
        return "info";
    case spdlog::level::warn:
        return "warn";
    case spdlog::level::err:
        return "error";
    case spdlog::level::critical:
        return "critical";
    case spdlog::level::off:
        return "off";
    default:
        spdlog::error("Unknown log level: {}. Using 'info'.", logLevel);
        return "info";
    }
}
} // namespace

MuscleCamera::MuscleCamera(
    int imageWidth,
    int imageHeight,
    int xOffset,
    int yOffset,
    double rollingShutterLineTimeUs,
    double sensorReadoutTimeUs,
    const RecorderConfig &recorderConfig,
    std::string profileDir,
    spdlog::level::level_enum logLevel)
    : x0_(xOffset + 1), x1_(xOffset + imageWidth), y0_(yOffset + 1),
      y1_(yOffset + imageHeight),
      rollingShutterLineTimeUs_(rollingShutterLineTimeUs),
      sensorReadoutTimeUs_(sensorReadoutTimeUs), imageWidth_(imageWidth),
      imageHeight_(imageHeight), recorderConfig_(recorderConfig),
      pcoCameraServerPID_(-1), frameDataPtr_(nullptr),
      shutterOpenTimePtr_(nullptr), mutexPtr_(nullptr), condVarPtr_(nullptr),
      lastFrameCount_(UINT_MAX) {
    if (!isROIValid()) {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    pid_t pid = fork(); // DANGEROUS! Pay special attention to avoid fork bomb

    if (pid < 0) {
        std::string errorMessage =
            "Failed to fork process in order to start PCO camera server: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    } else if (pid == 0) {
        // Child process: resolve pco-camera-server alongside the running
        // recorder binary so we always launch the matching build, rather
        // than whatever happens to be on $PATH.
        std::filesystem::path serverPath =
            std::filesystem::canonical("/proc/self/exe").parent_path() /
            "pco-camera-server";

        execl(
            serverPath.c_str(),
            "pco-camera-server",
            "--profile-dir",
            profileDir.c_str(),
            "--x-min",
            std::to_string(x0_).c_str(),
            "--x-max",
            std::to_string(x1_).c_str(),
            "--y-min",
            std::to_string(y0_).c_str(),
            "--y-max",
            std::to_string(y1_).c_str(),
            "--delay",
            "0", // sync delay is implemented in Arduino code, not here!
            "--verbosity",
            logLevelToStr(logLevel).c_str(),
            (char *)nullptr);

        // If execl returns, it must have failed
        std::string errorMessage = "Failed to execute PCO camera server at " +
                                   serverPath.string() + ": " +
                                   std::string(strerror(errno));
        spdlog::critical(errorMessage);
        exit(EXIT_FAILURE); // Exit child process
    } else {
        // Parent process
        pcoCameraServerPID_ = pid;
        spdlog::info(
            "PCO camera server started with process ID (PID): {}",
            pcoCameraServerPID_);

        // Wait for the camera server to initialize
        sleep(1); // sleep for 1 second

        // Setup shared memory buffers
        spdlog::info("Muscle camera API: Setting up shared memory buffer...");

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame data");
        bool createNew = false;

        size_t frameBufferSize = imageWidth * imageHeight * 2; // CV_16UC1
        std::string shmFrameDataName = recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_data_name");
        PCOSharedMemory::setupFrameData(
            shmFrameDataName, frameBufferSize, frameDataPtr_, createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for shutter-open "
            "time");
        std::string shmShutterOpenTimeName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_shutter_open_time_name");
        PCOSharedMemory::setupShutterOpenTime(
            shmShutterOpenTimeName, shutterOpenTimePtr_, createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame metadata");
        std::string shmFrameMetadataName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_frame_metadata_name");
        PCOSharedMemory::setupFrameMetadata(
            shmFrameMetadataName, frameMetadataPtr_, createNew);

        spdlog::info("Muscle camera API: Setting up shared memory for mutex");
        std::string shmMutexName = recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_mutex_name");
        PCOSharedMemory::setupMutex(shmMutexName, mutexPtr_, createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for cond var");
        std::string shmCondVarName = recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_condition_variable_name");
        PCOSharedMemory::setupConditionVariable(
            shmCondVarName, condVarPtr_, createNew);
        spdlog::info("Shared memory setup complete for PCO camera");
    }
}

MuscleCamera::~MuscleCamera() {
    // Cleanup code if needed
    if (pcoCameraServerPID_ > 0) {
        kill(pcoCameraServerPID_, SIGTERM);
        waitpid(pcoCameraServerPID_, nullptr, 0);
        spdlog::info("PCO camera server process terminated.");
    } else {
        spdlog::error(
            "Invalid process ID (PID) for PCO camera server: {}. "
            "PCO camera server process not started or already stopped. "
            "This should never happen.",
            pcoCameraServerPID_);
    }
}

FrameData MuscleCamera::waitForOneFrame() {
    while (true) {
        // Read data from shared memory
        pthread_mutex_lock(mutexPtr_);
        // spdlog::debug("Waiting for new frame...");
        pthread_cond_wait(condVarPtr_, mutexPtr_);
        // spdlog::debug("New frame available");
        unsigned int frameCount = frameMetadataPtr_->frameCount;
        uint64_t acquisitionTime = frameMetadataPtr_->acquisitionTime;
        cv::Mat image(imageHeight_, imageWidth_, CV_16UC1, frameDataPtr_);
        if (image.empty()) {
            spdlog::error("muscleCamera API got an empty image");
        }
        pthread_mutex_unlock(mutexPtr_);

        if (frameCount == lastFrameCount_) {
            spdlog::warn(
                "PCO camera API is waken up by the camera server, but no new "
                "frame is available. This could be a spurious wakeup of the "
                "condition variable (very rare), but more likely it indicates "
                "a problem in shared memory or synchronization primitives.");
            continue;
        } else {
            lastFrameCount_ = frameCount;
            FrameData frameData;
            frameData.acquisitionTime = acquisitionTime;
            frameData.receivedTime = getCurrentTimeMicroseconds();
            frameData.image = image;
            return frameData;
        }
    }
}

bool MuscleCamera::isROIValid() {
    int fullFrameWidth =
        recorderConfig_.getParameter<int>("muscle_camera", "full_frame_width");
    int fullFrameHeight =
        recorderConfig_.getParameter<int>("muscle_camera", "full_frame_height");
    if (x0_ < 1 || x1_ > fullFrameWidth || y0_ < 1 || y1_ > fullFrameHeight ||
        x0_ >= x1_ || y0_ >= y1_ || imageWidth_ % 32 != 0 ||
        imageHeight_ % 8 != 0 || imageWidth_ < 64 || imageHeight_ < 16) {
        spdlog::critical(
            "Invalid ROI for muscle camera. The following conditions must be "
            "met: 1 <= x0 < x1 <= {}; 1 <= y0 < y1 <= {}. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels. The width must be a "
            "multiple of 32 and the height must be a multiple of 8.",
            imageWidth_,
            imageHeight_);
        return false;
    }

    return true;
}

void MuscleCamera::setLightOnTime(unsigned int lightOnTimeMicrosecs) {
    if (shutterOpenTimePtr_ != nullptr) {
        int shutterOnTimeUs = calculateMuscleShutterOpenTime(
            imageHeight_, rollingShutterLineTimeUs_, lightOnTimeMicrosecs);
        *shutterOpenTimePtr_ = shutterOnTimeUs;
    } else {
        spdlog::error(
            "Cannot set exposure time. Shared memory pointer is null.");
    }
}

pid_t MuscleCamera::getCameraServerPID() const {
    return pcoCameraServerPID_;
}

int MuscleCamera::getNumLinesScanned() const {
    return imageHeight_;
}

int roundToNearestValidMuscleCamHorizontal(int value) {
    int remainder = value % 32;
    return value - remainder + (remainder < 16 ? 0 : 32);
}

int roundToNearestValidMuscleCamVertical(int value) {
    int remainder = value % 8;
    return value - remainder + (remainder < 4 ? 0 : 8);
}

bool DualRecordingConfig::computeParameters(
    int muscleImageHeight,
    double muscleCameraLineScanTimeUs,
    int muscleCameraReadoutTimeUs) {
    int behaviorIntervalUs = 1000000 / behaviorCameraFPS_;
    double muscleCameraFPS = behaviorCameraFPS_ / double(syncRatio_);
    int muscleIntervalUs = 1000000 / muscleCameraFPS;
    int rollingTimeUs = muscleImageHeight * muscleCameraLineScanTimeUs;
    if (2 * rollingTimeUs + muscleCameraReadoutTimeUs + muscleLightOnTimeUs_ >
        muscleIntervalUs) {
        spdlog::critical(
            "Computed muscle camera parameters are invalid: "
            "rollingTimeUs + muscleCameraReadoutTimeUs + muscleLightOnTimeUs_ "
            "must be less than or equal to muscleIntervalUs. "
            "rollingTimeUs = {}, "
            "muscleCameraReadoutTimeUs = {}, "
            "muscleLightOnTimeUs = {}, "
            "muscleIntervalUs = {}",
            rollingTimeUs,
            muscleCameraReadoutTimeUs,
            muscleLightOnTimeUs_,
            muscleIntervalUs);
        return false; // Invalid configuration
    }
    muscleShutterOpenTimeUs_ = rollingTimeUs + muscleLightOnTimeUs_;
    muscleCamTriggerDelayUs_ = muscleIntervalUs - rollingTimeUs;
    int minMuscleIntervalUs =
        muscleShutterOpenTimeUs_ + muscleCameraReadoutTimeUs;

    hasBeenChecked_ = true;
    return muscleIntervalUs >= minMuscleIntervalUs;
}

void DualRecordingConfig::saveToFile(const std::string &yamlPath) {
    if (!hasBeenChecked_) {
        spdlog::error(
            "DualRecordingConfig::saveToFile called before parameters were "
            "computed. Call computeParameters() first.");
        return;
    }

    YAML::Node config;

    // Save the primary configuration parameters
    config["record_both"] = recordBoth_;
    config["behavior_camera_fps"] = behaviorCameraFPS_;
    config["sync_ratio"] = syncRatio_;
    config["muscle_light_on_time_us"] = muscleLightOnTimeUs_;

    // Save the computed parameters
    config["muscle_shutter_open_time_us"] = muscleShutterOpenTimeUs_;
    config["muscle_cam_trigger_delay_us"] = muscleCamTriggerDelayUs_;

    // Create any parent directories if they don't exist
    std::filesystem::path filePath(yamlPath);
    if (auto dir = filePath.parent_path(); !dir.empty()) {
        std::filesystem::create_directories(dir);
    }

    // Write to file
    try {
        YAML::Emitter out;
        out << config;

        std::ofstream fout(yamlPath);
        if (!fout.is_open()) {
            spdlog::error("Failed to open file for writing: {}", yamlPath);
            return;
        }

        fout << out.c_str();
        fout.close();

        spdlog::info(
            "Dual recording timing configuration saved to {}", yamlPath);
    } catch (const std::exception &e) {
        spdlog::error(
            "Error saving timing configuration to {}: {}", yamlPath, e.what());
    }
}
