#include "recorder/apps/pco_camera_server.h"

namespace PCOCameraServer {
void printHelp(const char *programName) {
    // clang-format off
    std::cout
        << "Usage: " << programName << " [OPTIONS]\n"
        << "Options:\n"
        << "  -h,  --help              Display this help message\n"
        << "  -p,  --profile-dir PATH  Path to profile directory (default: ~/Spotlight/default/)\n"
        << "  -x0, --x-min X_MIN       x_min coordinate of the region of interest (default: 1)\n"
        << "  -x1, --x-max X_MAX       x_max coordinate of the region of interest (default: 1)\n"
        << "  -y0, --y-min Y_MIN       y_min coordinate of the region of interest (default: 2048)\n"
        << "  -y1, --y-max Y_MAX       y_max coordinate of the region of interest (default: 2048)\n"
        << "  -d,  --delay DELAY       Delay of shutter-open after trigger in microseconds (default: 0)\n"
        << "  -v,  --verbose           Enable verbose output (debug level)\n"
        << "  --verbosity LEVEL        Set verbosity level (trace, debug, info, warn, error, critical, off)\n"
        << std::endl;
    // clang-format on
}

spdlog::level::level_enum parseLogLevel(const std::string &level) {
    if (level == "trace")
        return spdlog::level::trace;
    if (level == "debug")
        return spdlog::level::debug;
    if (level == "info")
        return spdlog::level::info;
    if (level == "warn")
        return spdlog::level::warn;
    if (level == "error")
        return spdlog::level::err;
    if (level == "critical")
        return spdlog::level::critical;
    if (level == "off")
        return spdlog::level::off;

    spdlog::error("Unknown log level: {}. Using 'info'.", level);
    return spdlog::level::info;
}

CLIOptions parseCLI(int argc, char **argv) {
    CLIOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            std::exit(0);
        } else if (arg == "-v" || arg == "--verbose") {
            options.logLevel = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            options.logLevel = parseLogLevel(argv[++i]);
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc) {
            options.profileDir = argv[++i];
        } else if ((arg == "-x0" || arg == "--x-min") && i + 1 < argc) {
            options.x0 = std::stoi(argv[++i]);
        } else if ((arg == "-x1" || arg == "--x-max") && i + 1 < argc) {
            options.x1 = std::stoi(argv[++i]);
        } else if ((arg == "-y0" || arg == "--y-min") && i + 1 < argc) {
            options.y0 = std::stoi(argv[++i]);
        } else if ((arg == "-y1" || arg == "--y-max") && i + 1 < argc) {
            options.y1 = std::stoi(argv[++i]);
        } else if ((arg == "-d" || arg == "--delay") && i + 1 < argc) {
            options.delayUs = std::stoi(argv[++i]);
        } else if (arg[0] == '-') {
            spdlog::critical("Unknown option: {}", arg);
            printHelp(argv[0]);
            std::exit(1);
        } else if (i == 1 && arg[0] != '-') {
            // Support for positional argument (for backward compatibility)
            options.profileDir = arg;
        } else {
            spdlog::critical("Unknown option: {}", arg);
            printHelp(argv[0]);
            std::exit(1);
        }
    }

    return options;
}

std::string expandPath(const std::string &path) {
    // Check if the path starts with "~/"
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        // Get the HOME environment variable
        const char *homeDir = std::getenv("HOME");

        // If HOME is available, replace "~/" with the home directory
        if (homeDir) {
            std::filesystem::path expandedPath =
                std::filesystem::path(homeDir) / path.substr(2);
            return expandedPath.string();
        } else {
            spdlog::error(
                "Failed to expand ~ in directory path '{}' because $HOME is "
                "not defined. Set the $HOME environment variable or use "
                "absolute path.",
                path.c_str());
        }
    }

    // Return the original path if it doesn't start with "~/"
    return path;
}

// Signal handler function
void signalHandler(int signal) {
    const char *signalName = signal == SIGINT    ? "SIGINT"
                             : signal == SIGTERM ? "SIGTERM"
                                                 : "Unknown signal";
    spdlog::info(
        "Shutdown signal received ({}: {}). Cleaning up and exiting...",
        signal,
        signalName);
    shutdownRequested.store(true);
}

uint64_t getCurrentTimeMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

void setupPCOCamera(
    pco::Camera &camera,
    unsigned int defaultShutterOpenTimeUs,
    unsigned int x0,
    unsigned int x1,
    unsigned int y0,
    unsigned int y1,
    unsigned int delayUs,
    unsigned int fullFrameWidth,
    unsigned int fullFrameHeight) {
    // Set configuration
    spdlog::info("Getting default PCO camera configuration");
    camera.defaultConfiguration();
    pco::Configuration config = camera.getConfiguration();
    config.roi.x0 = x0;
    config.roi.x1 = x1;
    config.roi.y0 = y0;
    config.roi.y1 = y1;
    // Auto-sequence ("auto trigger") = continuous rolling shutter: the camera
    // free-runs, exposing each line back-to-back with no idle line-reset time,
    // instead of waiting for an external TTL trigger per frame. This is required
    // by the acquisition design (docs/data_acquisition.md): the trigger firmware
    // does NOT trigger this camera -- it locks the behavior camera to the muscle
    // camera's free-running common-time signal on SMA #4 (configured below). With
    // TRIGGER_MODE_EXTERNALTRIGGER the camera would wait forever for a trigger the
    // firmware never sends, never expose, never drive SMA #4, and the firmware
    // would in turn wait forever for the common-time onset -- freezing both
    // cameras. The free-run frame rate is set via the nominal exposure (see
    // setExposureTime below and the acquisition loop's live exposure updates).
    config.trigger_mode = TRIGGER_MODE_AUTOTRIGGER;
    config.acquire_mode = ACQUIRE_MODE_AUTO;
    // Zero inter-frame delay keeps the rolling shutter continuous (no idle time).
    // Any sync delay is implemented in the trigger firmware, not here.
    config.delay_time_s = delayUs / 1000000.0; // Convert to seconds
    config.noise_filter_mode = NOISE_FILTER_MODE_ON;
    // config.timestamp_mode = TIMESTAMP_MODE_ASCII;
    spdlog::info("Setting PCO camera configuration");
    camera.setConfiguration(config);
    spdlog::info("PCO camera configuration set");
    spdlog::info(
        "PCO camera configuration set: "
        "x0={}, x1={}, y0={}, y1={}, delay_us={}",
        config.roi.x0,
        config.roi.x1,
        config.roi.y0,
        config.roi.y1,
        delayUs);

    // Set exposure time
    spdlog::info(
        "Setting PCO camera shutter-open time to {} us",
        defaultShutterOpenTimeUs);
    camera.setExposureTime(defaultShutterOpenTimeUs / 1000000.0);
    camera.autoExposureOff();
    spdlog::info("PCO camera shutter-open time set");

    // Set trigger polarity
    spdlog::info("Setting PCO camera trigger polarity to rising edge");
    camera.configureHWIO_1_exposureTrigger(
        true, pco::HWIO_EdgePolarity::rising_edge);

    // Drive SMA #4 as the muscle camera's "common time" reference for the
    // trigger firmware. The firmware (trigger_firmware: DeviceIO::isMuscCommonTime)
    // treats the line being HIGH as "in common time" and fires the behavior frame
    // + blue LED on the LOW->HIGH onset, so the camera must drive the line HIGH for
    // exactly the common-time window.
    //
    // - signal_type status_expos: report the exposure status on SMA #4.
    // - timing global: for a rolling shutter, "global" is the interval when all
    //   lines are exposed simultaneously, i.e. the common time (see
    //   docs/data_acquisition.md). NOT all_lines, which spans the whole rolling
    //   exposure envelope and would make the onset fire ~rollingTime too early.
    // - polarity high_level: status_expos is asserted during the global window, so
    //   high_level makes the line HIGH during common time (LOW otherwise), matching
    //   the firmware's edge.
    camera.configureHWIO_4_statusExpos(
        true,
        pco::HWIO_Polarity::high_level,
        pco::HWIO_4_SignalType::status_expos,
        pco::HWIO_StatusExpos_Timing::global); // global = common time
}

void serveFrames(
    const std::string &shmFrameDataName,
    const size_t frameBufferSize,
    const std::string &shmShutterOpenTimeName,
    const std::string &shmFrameMetadataName,
    const std::string &shmMutexName,
    const std::string &shmCondVarName,
    const unsigned int defaultShutterOpenTimeUs,
    const unsigned int x0,
    const unsigned int x1,
    const unsigned int y0,
    const unsigned int y1,
    const unsigned int delayUs,
    const unsigned int fullFrameWidth,
    const unsigned int fullFrameHeight) {
    // Setup shared memory buffers
    bool createNew = true;

    spdlog::info("PCO camera server: Setting up shared memory buffers...");

    spdlog::info("PCO camera server: Setting up shared memory for frame data");
    uint8_t *frameDataPtr;
    PCOSharedMemory::setupFrameData(
        shmFrameDataName, frameBufferSize, frameDataPtr, createNew);

    spdlog::info(
        "PCO camera server: Setting up shared memory for exposure time");
    unsigned int *shutterOpenTimePtr;
    PCOSharedMemory::setupShutterOpenTime(
        shmShutterOpenTimeName, shutterOpenTimePtr, createNew);

    spdlog::info(
        "PCO camera server: Setting up shared memory for frame metadata");
    PCOSharedMemory::FrameMetadata *frameMetadataPtr;
    PCOSharedMemory::setupFrameMetadata(
        shmFrameMetadataName, frameMetadataPtr, createNew);

    spdlog::info("PCO camera server: Setting up shared memory for mutex");
    pthread_mutex_t *mutexPtr;
    PCOSharedMemory::setupMutex(shmMutexName, mutexPtr, createNew);

    spdlog::info("PCO camera server: Setting up shared memory for cond var");
    pthread_cond_t *condVarPtr;
    PCOSharedMemory::setupConditionVariable(
        shmCondVarName, condVarPtr, createNew);

    spdlog::info("PCO camera server: Shared memory setup complete");

    // Set default exposure time and initial frame count
    spdlog::info("Setting default exposure time in shared memory");
    *shutterOpenTimePtr = defaultShutterOpenTimeUs;

    // Initialize PCO camera
    spdlog::info("Setting up PCO camera");
    pco::Camera camera;
    PCOCameraServer::setupPCOCamera(
        camera,
        defaultShutterOpenTimeUs,
        x0,
        x1,
        y0,
        y1,
        delayUs,
        fullFrameWidth,
        fullFrameHeight);
    spdlog::info("PCO camera setup complete");

    // Create local data holders
    pco::Image pcoImage;
    cv::Mat cvImage;
    bool isFirstFrame = true;
    unsigned int frameCount = 0;

    // Start camera acquisition
    spdlog::info("Starting PCO camera acquisition");
    int bufferSize = 10;
    camera.record(bufferSize, pco::RecordMode::ring_buffer);
    spdlog::info("Recording mode set to ring buffer with size {}", bufferSize);

    unsigned int currentExposureTimeUs = defaultShutterOpenTimeUs;

    // Data acquisition loop
    spdlog::info("PCO camera server starting its data acquisition loop");
    while (!shutdownRequested.load()) {
        // Check if we should change exposure time
        unsigned int targetExposureTime = *shutterOpenTimePtr;
        if (targetExposureTime != currentExposureTimeUs) {
            spdlog::info(
                "PCO camera server is changing exposure time to {} us",
                targetExposureTime);
            camera.setExposureTime(targetExposureTime / 1000000.0);
            currentExposureTimeUs = targetExposureTime;
            spdlog::info(
                "Changed exposure time to {} us", currentExposureTimeUs);
        }

        // Wait for new frame to arrive
        // Note: If muscle triggers are very slow or disabled (i.e. period
        // set to INT_MAX in case the "Enable muscle imaging" option is
        // unchecked), we might be forever blocked in the waitForFirstImage
        // or waitForNewImage function. Thus, when the user requests the
        // program to stop (by sending a SIGINT or SIGTERM), the program
        // will ignore it. To avoid this, we add a small timeout to the
        // calls and wrap put them in an infinite loop. This way, we still
        // wait indefinitely for new frames to come, but once in a while we
        // move on to the next iteration of the inner loop, which gives us
        // a chance to check if shutdownRequested has been set to true and
        // break accordingly.
        // spdlog::debug("Entering waiting inner loop");
        while (true) {
            if (isFirstFrame) {
                try {
                    camera.waitForFirstImage(
                        WAIT_WITH_SMALL_DELAY, WAIT_TIMEOUT_SECS);
                    isFirstFrame = false;
                    // spdlog::debug(
                    //     "First frame received, breaking out of waiting "
                    //     "inner loop");
                    break;
                } catch (pco::CameraException &e) {
                    uint32_t errorCode = e.error_code();
                    // spdlog::debug(
                    //     "Exception while waiting for first image; "
                    //     "error code 0x{0:08x}",
                    //     errorCode);
                    if (errorCode == TIMEOUT_ERROR_CODE) {
                        // This is expected, so do nothing
                    } else {
                        throw;
                    }
                }
            } else {
                try {
                    camera.waitForNewImage(
                        WAIT_WITH_SMALL_DELAY, WAIT_TIMEOUT_SECS);
                    // spdlog::debug(
                    //     "New frame received, breaking out of waiting "
                    //     "inner loop");
                    break;
                } catch (pco::CameraException &e) {
                    uint32_t errorCode = e.error_code();
                    // spdlog::debug(
                    //     "Exception while waiting for first image; "
                    //     "error code 0x{0:08x}",
                    //     errorCode);
                    if (errorCode == TIMEOUT_ERROR_CODE) {
                        // This is expected, so do nothing
                    } else {
                        throw;
                    }
                }
            }
            if (shutdownRequested.load()) {
                spdlog::info(
                    "PCO camera server: Shutdown requested, breaking out "
                    "of inner waiting loop");
                break;
            }
        }
        if (shutdownRequested.load()) {
            break;
        }

        // Fetch image and convert to OpenCV format
        // spdlog::debug("PCO camera server got new frame. Serving.");
        camera.image(
            pcoImage, PCO_RECORDER_LATEST_IMAGE, pco::DataFormat::Mono16);
        cvImage = cv::Mat(
            pcoImage.height(),
            pcoImage.width(),
            CV_16UC1,
            pcoImage.raw_data().first);

        // Gather metadata
        PCOSharedMemory::FrameMetadata frameMetadata;
        frameMetadata.frameCount = frameCount++;
        frameMetadata.acquisitionTime =
            PCOCameraServer::getCurrentTimeMicroseconds();

        // Mutex-protected zone! Updata image buffer and frame count
        pthread_mutex_lock(mutexPtr);
        memcpy(frameDataPtr, cvImage.data, frameBufferSize);
        memcpy(
            frameMetadataPtr,
            &frameMetadata,
            sizeof(PCOSharedMemory::FrameMetadata));
        pthread_cond_signal(condVarPtr);
        // spdlog::debug("PCO camera server signaled new frame");
        pthread_mutex_unlock(mutexPtr);
    }

    camera.stop();
    spdlog::info("PCO camera stopped.");
}
} // namespace PCOCameraServer

int main(int argc, char *argv[]) {
    std::signal(SIGINT, PCOCameraServer::signalHandler);
    std::signal(SIGTERM, PCOCameraServer::signalHandler);

    PCOCameraServer::CLIOptions options = PCOCameraServer::parseCLI(argc, argv);
    spdlog::set_level(options.logLevel);

    std::filesystem::path profileDir =
        std::filesystem::path(PCOCameraServer::expandPath(options.profileDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info(
        "pcoCameraServer loading recorder configuration from {}",
        configPath.string());
    RecorderConfig recorderConfig(configPath);

    const unsigned int fullFrameWidth =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_width");
    const unsigned int fullFrameHeight =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_height");
    unsigned int roiWidth = options.x1 - options.x0 + 1;
    unsigned int roiHeight = options.y1 - options.y0 + 1;

    // Initial nominal per-line exposure for the free-running (auto-sequence)
    // camera. In continuous mode the exposure sets the frame rate
    // (rate = 1/(exposure + readout)), so derive it from the default streaming
    // muscle interval (streaming sync ratio / streaming behavior FPS):
    //   exposure = muscleInterval - readout.
    // The recorder GUI overwrites this live (via the shared shutter-open-time
    // region) as soon as it knows the active streaming/recording parameters.
    const double sensorReadoutTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    const int streamingBehFPS = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    const int streamingSyncRatio = recorderConfig.getParameter<int>(
        "muscle_camera", "streaming_sync_ratio");
    const unsigned int defaultMuscleIntervalUs = static_cast<unsigned int>(
        1000000.0 * streamingSyncRatio / streamingBehFPS);
    const unsigned int defaultShutterOpenTimeUs = defaultMuscleIntervalUs -
        static_cast<unsigned int>(sensorReadoutTimeUs);

    // Validate image dimensions
    if (options.x0 == 0 || options.y0 == 0 || options.x1 > fullFrameWidth ||
        options.y1 > fullFrameHeight || options.x0 >= options.x1 ||
        options.y0 >= options.y1) {
        spdlog::critical(
            "Invalid image dimensions. The following is required: "
            "0 < x0 < x1 <= {}; 0 < y0 < y1 <= {}.",
            fullFrameHeight,
            fullFrameWidth);
        return 1;
    }

    if (roiWidth % 32 != 0 || roiHeight % 8 != 0 || roiWidth < 64 ||
        roiHeight < 16) {
        spdlog::critical(
            "Invalid ROI for muscle camera. ROI width must be a multiple of "
            "32 and ROI height must be a multiple of 8. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels.");
        return 1;
    }

    // Compute buffer size for each frame
    const size_t sizePerPixel = 2; // CV_16UC1
    const size_t frameBufferSize = roiWidth * roiHeight * sizePerPixel;

    // Set up shared memory buffers for frame data, mutex, and semaphore
    const std::string shmFrameDataName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_data_name");
    const std::string shmShutterOpenTimeName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_shutter_open_time_name");
    const std::string shmFrameMetadataName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_metadata_name");
    const std::string shmMutexName = recorderConfig.getParameter<std::string>(
        "muscle_camera", "shared_mutex_name");
    const std::string shmCondVarName = recorderConfig.getParameter<std::string>(
        "muscle_camera", "shared_condition_variable_name");

    // The PCO SDK keeps global state (camera scan/open handles, recorder, etc.)
    // that must be initialized before any pco::Camera is constructed. Skipping
    // this makes the Camera constructor's PCO_ScanCameras/PCO_OpenCameraDevice
    // calls operate on an invalid SDK handle, which surfaces as
    // "SDK DLL error 0xa00a3002 ... Handle is invalid." Mirror the PCO samples,
    // which always pair PCO_InitializeLib()/PCO_CleanupLib() around camera use.
    spdlog::info("Initializing PCO SDK library");
    if (int err = PCO_InitializeLib(); err != PCO_NOERROR) {
        spdlog::critical(
            "Failed to initialize PCO SDK library (error 0x{:08x})",
            static_cast<uint32_t>(err));
        return 1;
    }

    try {
        PCOCameraServer::serveFrames(
            shmFrameDataName,
            frameBufferSize,
            shmShutterOpenTimeName,
            shmFrameMetadataName,
            shmMutexName,
            shmCondVarName,
            defaultShutterOpenTimeUs,
            options.x0,
            options.x1,
            options.y0,
            options.y1,
            options.delayUs,
            fullFrameWidth,
            fullFrameHeight);
    } catch (pco::CameraException &e) {
        spdlog::critical(
            "PCO camera server aborting due to camera error (0x{:08x}): {}",
            static_cast<uint32_t>(e.error_code()),
            e.what());
        PCO_CleanupLib();
        return 1;
    }

    PCO_CleanupLib();
    spdlog::info("PCO camera server stopping...");
    return 0;
}