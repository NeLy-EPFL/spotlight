#include "recorder/peripherals/muscle_camera.h"

#include <cerrno> // errno, ECHILD
#include <chrono>
#include <filesystem>
#include <sys/prctl.h> // prctl, PR_SET_PDEATHSIG
#include <thread>

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
    // Member initializers are in declaration order (avoids -Wreorder).
    : x0_(xOffset + 1), x1_(xOffset + imageWidth), y0_(yOffset + 1),
      y1_(yOffset + imageHeight), imageWidth_(imageWidth),
      imageHeight_(imageHeight),
      rollingShutterLineTimeUs_(rollingShutterLineTimeUs),
      sensorReadoutTimeUs_(sensorReadoutTimeUs), pcoCameraServerPID_(-1),
      frameDataPtr_(nullptr), shutterOpenTimePtr_(nullptr),
      frameMetadataPtr_(nullptr), mutexPtr_(nullptr), condVarPtr_(nullptr),
      recorderConfig_(recorderConfig), lastFrameCount_(UINT_MAX) {
    if (!isROIValid()) {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    // Capture our PID before forking so the child can detect (after arming its
    // parent-death signal below) whether we already died in the race window
    // between fork() and prctl().
    pid_t parentPidBeforeFork = getpid();

    pid_t pid = fork(); // DANGEROUS! Pay special attention to avoid fork bomb

    if (pid < 0) {
        std::string errorMessage =
            "Failed to fork process in order to start PCO camera server: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    } else if (pid == 0) {
        // Child process.
        //
        // Ask the kernel to send us SIGTERM if our parent (the recorder) dies.
        // Without this, a recorder that is SIGKILLed or crashes never runs
        // ~MuscleCamera(), so the camera server is orphaned and keeps the PCO
        // camera open indefinitely. The next run then fails to open the camera
        // (it is already "attached") and the SDK reports the cryptic
        // "Handle is invalid" (0xa00a3002). The server installs a SIGTERM
        // handler that stops and closes the camera cleanly. PR_SET_PDEATHSIG
        // survives the execl() below because pco-camera-server is not set-uid.
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        // Close the race where the parent already died before the prctl() above
        // took effect: in that case exit now rather than becoming an orphan.
        if (getppid() != parentPidBeforeFork) {
            _exit(EXIT_FAILURE);
        }

        // Resolve pco-camera-server alongside the running recorder binary so we
        // always launch the matching build, rather than whatever happens to be
        // on $PATH.
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

void MuscleCamera::stop() {
    // Terminate the PCO camera server process. This is bounded: an unresponsive
    // server can never block shutdown indefinitely, because we escalate to
    // SIGKILL if it does not exit within the grace period. The process is reaped
    // in both paths so it does not linger as a zombie.
    //
    // Idempotent: pcoCameraServerPID_ is cleared once reaped, so a later call
    // (e.g. an explicit stop() followed by the destructor) is a no-op.
    if (pcoCameraServerPID_ <= 0) {
        return;
    }

    pid_t pid = pcoCameraServerPID_;
    pcoCameraServerPID_ = -1;

    kill(pid, SIGTERM);

    // Poll for graceful exit up to a bounded deadline before escalating. The
    // server checks its shutdown flag once per frame-wait timeout (0.1 s), so
    // it normally exits well within this window.
    constexpr int gracePeriodMs = 3000;
    constexpr int pollIntervalMs = 20;
    bool reaped = false;
    for (int elapsedMs = 0; elapsedMs < gracePeriodMs;
         elapsedMs += pollIntervalMs) {
        pid_t result = waitpid(pid, nullptr, WNOHANG);
        if (result == pid || (result == -1 && errno == ECHILD)) {
            reaped = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }

    if (!reaped) {
        spdlog::warn(
            "PCO camera server (PID {}) did not exit within {} ms of SIGTERM; "
            "escalating to SIGKILL.",
            pid,
            gracePeriodMs);
        kill(pid, SIGKILL);
        // SIGKILL cannot be caught or ignored, so this blocking reap is bounded.
        waitpid(pid, nullptr, 0);
    }

    spdlog::info("PCO camera server process terminated.");
}

MuscleCamera::~MuscleCamera() {
    stop();
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

        if (frameCount == lastFrameCount_) {
            pthread_mutex_unlock(mutexPtr_);
            spdlog::warn(
                "PCO camera API is waken up by the camera server, but no new "
                "frame is available. This could be a spurious wakeup of the "
                "condition variable (very rare), but more likely it indicates "
                "a problem in shared memory or synchronization primitives.");
            continue;
        }

        // Copy the frame out of shared memory while still holding the lock. The
        // cv::Mat below only wraps frameDataPtr_, which the camera server
        // overwrites (memcpy) on every new frame; cloning under the lock takes a
        // private copy before the server can begin writing the next frame, so
        // the returned image can never be torn by a concurrent write.
        cv::Mat image =
            cv::Mat(imageHeight_, imageWidth_, CV_16UC1, frameDataPtr_).clone();
        pthread_mutex_unlock(mutexPtr_);

        if (image.empty()) {
            spdlog::error("muscleCamera API got an empty image");
        }

        lastFrameCount_ = frameCount;
        FrameData frameData;
        frameData.acquisitionTime = acquisitionTime;
        frameData.receivedTime = getCurrentTimeMicroseconds();
        frameData.image = image;
        return frameData;
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

void MuscleCamera::setNominalExposureUs(unsigned int exposureUs) {
    if (shutterOpenTimePtr_ != nullptr) {
        // The PCO camera server polls this shared value in its acquisition loop
        // and applies it as the camera's nominal per-line exposure (see
        // serveFrames() in pco_camera_server_main.cc). In continuous mode this
        // also sets the free-run frame rate.
        *shutterOpenTimePtr_ = exposureUs;
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

bool MuscleTriggerTiming::computeParameters(
    int muscleImageHeight,
    double muscleCameraLineScanTimeUs,
    int muscleCameraReadoutTimeUs) {
    double muscleCameraFPS = behaviorCameraFPS_ / double(syncRatio_);
    int muscleIntervalUs = 1000000 / muscleCameraFPS;
    int rollingTimeUs = muscleImageHeight * muscleCameraLineScanTimeUs;

    // Continuous rolling shutter (auto-sequence): the camera free-runs at
    // 1/(nominalExposure + readout). Set the nominal per-line exposure so one
    // frame fills the requested muscle interval, then split it into the rolling
    // time (line skew) and the common time (all lines exposing simultaneously).
    // The light-on window must fit inside the common time, leaving a
    // non-negative buffer:
    //   nominalExposure = muscleInterval - readout = rollingTime + commonTime
    //   commonTime      = lightOn + bufferTime
    // Valid only if rollingTime + lightOn + readout <= muscleInterval. (Note the
    // single rollingTime: triggered acquisition would need 2 * rollingTime, but
    // continuous rolling has no idle line-reset time -- see
    // docs/data_acquisition.md.)
    nominalExposureUs_ = muscleIntervalUs - muscleCameraReadoutTimeUs;
    commonTimeUs_ = nominalExposureUs_ - rollingTimeUs;
    bufferTimeUs_ = commonTimeUs_ - muscleLightOnTimeUs_;
    if (bufferTimeUs_ < 0) {
        spdlog::critical(
            "Computed muscle camera parameters are invalid: "
            "rollingTime + lightOn + readout must be <= muscleInterval. "
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
    return true;
}
