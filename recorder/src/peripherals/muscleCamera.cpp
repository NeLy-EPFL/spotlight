#include "muscleCamera.hpp"

namespace
{
    std::string logLevelToStr(spdlog::level::level_enum logLevel)
    {
        switch (logLevel)
        {
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
}

MuscleCamera::MuscleCamera(int imageWidth,
                           int imageHeight,
                           int xOffset,
                           int yOffset,
                           const RecorderConfig &recorderConfig,
                           std::string profileDir,
                           spdlog::level::level_enum logLevel)
    : x0_(xOffset + 1),
      x1_(xOffset + imageWidth),
      y0_(yOffset + 1),
      y1_(yOffset + imageHeight),
      imageWidth_(imageWidth),
      imageHeight_(imageHeight),
      recorderConfig_(recorderConfig),
      pcoCameraServerPID_(-1),
      frameDataPtr_(nullptr),
      exposureTimePtr_(nullptr),
      mutexPtr_(nullptr),
      condVarPtr_(nullptr),
      lastFrameCount_(UINT_MAX)
{
    if (!isROIValid())
    {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    pid_t pid = fork(); // DANGEROUS! Pay special attention to avoid fork bomb

    if (pid < 0)
    {
        std::string errorMessage =
            "Failed to fork process in order to start PCO camera server: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    else if (pid == 0)
    {
        // Child process
        execl("./pco-camera-server",
              "./pco-camera-server",
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
              "--verbosity",
              logLevelToStr(logLevel).c_str(),
              (char *)nullptr);

        // If execl returns, it must have failed
        std::string errorMessage =
            "Failed to execute PCO camera server: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        exit(EXIT_FAILURE); // Exit child process
    }
    else
    {
        // Parent process
        pcoCameraServerPID_ = pid;
        spdlog::info("PCO camera server started with process ID (PID): {}",
                     pcoCameraServerPID_);

        // Wait for the camera server to initialize
        sleep(1); // sleep for 1 second

        // Setup shared memory buffers
        spdlog::info("Muscle camera API: Setting up shared memory buffer...");

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame data");
        bool createNew = false;

        size_t frameBufferSize =
            imageWidth * imageHeight * 2; // CV_16UC1
        std::string shmFrameDataName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_frame_data_name");
        PCOSharedMemory::setupFrameData(shmFrameDataName,
                                        frameBufferSize,
                                        frameDataPtr_,
                                        createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for exposure time");
        std::string shmExposureTimeName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_exposure_time_name");
        PCOSharedMemory::setupExposureTime(shmExposureTimeName,
                                           exposureTimePtr_,
                                           createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame metadata");
        std::string shmFrameMetadataName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_frame_metadata_name");
        PCOSharedMemory::setupFrameMetadata(shmFrameMetadataName,
                                            frameMetadataPtr_,
                                            createNew);

        spdlog::info("Muscle camera API: Setting up shared memory for mutex");
        std::string shmMutexName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_mutex_name");
        PCOSharedMemory::setupMutex(shmMutexName, mutexPtr_,
                                    createNew);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for cond var");
        std::string shmCondVarName =
            recorderConfig.getParameter<std::string>(
                "muscle_camera", "shared_condition_variable_name");
        PCOSharedMemory::setupConditionVariable(shmCondVarName,
                                                condVarPtr_,
                                                createNew);
        spdlog::info("Shared memory setup complete for PCO camera");
    }
}

MuscleCamera::~MuscleCamera()
{
    // Cleanup code if needed
    if (pcoCameraServerPID_ > 0)
    {
        kill(pcoCameraServerPID_, SIGTERM);
        waitpid(pcoCameraServerPID_, nullptr, 0);
        spdlog::info("PCO camera server process terminated.");
    }
    else
    {
        spdlog::error(
            "Invalid process ID (PID) for PCO camera server: {}. "
            "PCO camera server process not started or already stopped. "
            "This should never happen.",
            pcoCameraServerPID_);
    }
}

FrameData MuscleCamera::waitForOneFrame()
{
    while (true)
    {
        // Read data from shared memory
        pthread_mutex_lock(mutexPtr_);
        // spdlog::debug("Waiting for new frame...");
        pthread_cond_wait(condVarPtr_, mutexPtr_);
        // spdlog::debug("New frame available");
        unsigned int frameCount = frameMetadataPtr_->frameCount;
        uint64_t acquisitionTime = frameMetadataPtr_->acquisitionTime;
        cv::Mat image(imageHeight_, imageWidth_, CV_16UC1, frameDataPtr_);
        if (image.empty())
        {
            spdlog::error("muscleCamera API got an empty image");
        }
        pthread_mutex_unlock(mutexPtr_);

        if (frameCount == lastFrameCount_)
        {
            spdlog::warn(
                "PCO camera API is waken up by the camera server, but no new "
                "frame is available. This could be a spurious wakeup of the "
                "condition variable (very rare), but more likely it indicates "
                "a problem in shared memory or synchronization primitives.");
            continue;
        }
        else
        {
            lastFrameCount_ = frameCount;
            FrameData frameData;
            frameData.acquisitionTime = acquisitionTime;
            frameData.receivedTime = getCurrentTimeMicroseconds();
            frameData.image = image;
            return frameData;
        }
    }
}

bool MuscleCamera::isROIValid()
{
    int fullFrameWidth = recorderConfig_.getParameter<int>(
        "muscle_camera", "full_frame_width");
    int fullFrameHeight = recorderConfig_.getParameter<int>(
        "muscle_camera", "full_frame_height");
    if (x0_ < 1 ||
        x1_ > fullFrameWidth ||
        y0_ < 1 ||
        y1_ > fullFrameHeight ||
        x0_ >= x1_ ||
        y0_ >= y1_ ||
        imageWidth_ % 32 != 0 ||
        imageHeight_ % 8 != 0 ||
        imageWidth_ < 64 ||
        imageHeight_ < 16)
    {
        spdlog::critical(
            "Invalid ROI for muscle camera. The following conditions must be "
            "met: 1 <= x0 < x1 <= {}; 1 <= y0 < y1 <= {}. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels. The width must be a "
            "multiple of 32 and the height must be a multiple of 8.",
            imageWidth_, imageHeight_);
        return false;
    }

    return true;
}

void MuscleCamera::setExposureTime(unsigned int exposureTimeMicrosecs)
{
    if (exposureTimePtr_ != nullptr)
    {
        *exposureTimePtr_ = exposureTimeMicrosecs;
    }
    else
    {
        spdlog::error(
            "Cannot set exposure time. Shared memory pointer is null.");
    }
}

pid_t MuscleCamera::getCameraServerPID() const
{
    return pcoCameraServerPID_;
}

int MuscleCamera::getNumLinesScanned() const
{
    return imageHeight_;
}

int roundToNearestValidMuscleCamHorizontal(int value)
{
    int remainder = value % 32;
    return value - remainder + (remainder < 16 ? 0 : 32);
}

int roundToNearestValidMuscleCamVertical(int value)
{
    int remainder = value % 8;
    return value - remainder + (remainder < 4 ? 0 : 8);
}