#include "pcoCameraServer.hpp"

namespace PCOCameraServer
{
    void printHelp(const char *programName)
    {
        std::cout
            << "Usage: " << programName << " [OPTIONS]\n"
            << "Options:\n"
            << "  -h, --help                 Display this help message\n"
            << "  -p, --profile-dir PATH     Path to profile directory (default: ~/Spotlight/default/)\n"
            << "  -W, --image-width WIDTH    Width of the image (default: 2048)\n"
            << "  -H, --image-height HEIGHT  Height of the image (default: 2048)\n"
            << "  -x, --x-offset OFFSET      X offset for the region of interest. If -1, one will be calculated automatically so that the image is centered (default: -1)\n"
            << "  -y, --y-offset OFFSET      Y offset for the region of interest. Same as x-offset (default: -1)\n"
            << "  -v, --verbose              Enable verbose output (debug level)\n"
            << "  --verbosity LEVEL          Set verbosity level (trace, debug, info, warn, error, critical, off)\n"
            << std::endl;
    }

    spdlog::level::level_enum parseLogLevel(const std::string &level)
    {
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

    CLIOptions parseCLI(int argc, char **argv)
    {
        CLIOptions options;

        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];

            if (arg == "-h" || arg == "--help")
            {
                printHelp(argv[0]);
                std::exit(0);
            }
            else if (arg == "-v" || arg == "--verbose")
            {
                options.logLevel = spdlog::level::debug;
            }
            else if (arg == "--verbosity" && i + 1 < argc)
            {
                options.logLevel = parseLogLevel(argv[++i]);
            }
            else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc)
            {
                options.profileDir = argv[++i];
            }
            else if ((arg == "-W" || arg == "--image-width") && i + 1 < argc)
            {
                options.imageWidth = std::stoi(argv[++i]);
            }
            else if ((arg == "-H" || arg == "--image-height") && i + 1 < argc)
            {
                options.imageHeight = std::stoi(argv[++i]);
            }
            else if ((arg == "-x" || arg == "--x-offset") && i + 1 < argc)
            {
                options.xOffset = std::stoi(argv[++i]);
            }
            else if ((arg == "-y" || arg == "--y-offset") && i + 1 < argc)
            {
                options.yOffset = std::stoi(argv[++i]);
            }
            else if (arg[0] == '-')
            {
                spdlog::critical("Unknown option: {}", arg);
                printHelp(argv[0]);
                std::exit(1);
            }
            else if (i == 1 && arg[0] != '-')
            {
                // Support for positional argument (for backward compatibility)
                options.profileDir = arg;
            }
            else
            {
                spdlog::critical("Unknown option: {}", arg);
                printHelp(argv[0]);
                std::exit(1);
            }
        }

        if (options.imageHeight == -1 || options.imageWidth == -1)
        {
            spdlog::critical(
                "Image dimensions not specified. Use -W and -H options.");
        }

        return options;
    }

    std::string expandPath(const std::string &path)
    {
        // Check if the path starts with "~/"
        if (path.size() >= 2 && path[0] == '~' && path[1] == '/')
        {
            // Get the HOME environment variable
            const char *homeDir = std::getenv("HOME");

            // If HOME is available, replace "~/" with the home directory
            if (homeDir)
            {
                std::filesystem::path expandedPath =
                    std::filesystem::path(homeDir) / path.substr(2);
                return expandedPath.string();
            }
            else
            {
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
    void signalHandler(int signal)
    {
        const char *signalName =
            signal == SIGINT ? "SIGINT" : signal == SIGTERM ? "SIGTERM"
                                                            : "Unknown signal";
        spdlog::info(
            "Shutdown signal received ({}: {}). Cleaning up and exiting...",
            signal, signalName);
        shutdownRequested.store(true);
    }

    uint64_t getCurrentTimeMicroseconds()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    }

    /**
     * @brief Sets up a PCO camera with specified configuration parameters
     *
     * Configures a PCO camera with specific settings including region of
     * interest (ROI), trigger mode, acquisition mode, and exposure time.
     * If the X or Y offsets are set to -1, they will be automatically
     * calculated to center the ROI on the sensor.
     *
     * @param camera Reference to the pco::Camera object to configure
     * @param defaultExposureTimeUs Default exposure time in microseconds
     * @param imageWidth Width of the image to capture in pixels
     * @param imageHeight Height of the image to capture in pixels
     * @param xOffset X offset for the ROI (use -1 for auto-centering)
     * @param yOffset Y offset for the ROI (use -1 for auto-centering)
     * @param fullFrameWidth Full width of the camera sensor in pixels
     * @param fullFrameHeight Full height of the camera sensor in pixels
     *
     * @note The function sets external trigger mode, auto acquisition mode,
     *       and enables noise filtering
     * @note ROI coordinates are 1-based in the PCO API (offsets have 1 added
     *       to them)
     */
    void setupPCOCamera(pco::Camera &camera,
                        unsigned int defaultExposureTimeUs,
                        unsigned int imageWidth,
                        unsigned int imageHeight,
                        int xOffset,
                        int yOffset,
                        unsigned int fullFrameWidth,
                        unsigned int fullFrameHeight)
    {
        if (xOffset == -1)
        {
            xOffset = calculateOffset(fullFrameWidth, imageWidth);
        }
        if (yOffset == -1)
        {
            yOffset = calculateOffset(fullFrameHeight, imageHeight);
        }

        // Set configuration
        spdlog::info("Setting up PCO camera");
        spdlog::info("Getting default PCO camera configuration");
        camera.defaultConfiguration();
        pco::Configuration config = camera.getConfiguration();
        config.roi.x0 = xOffset + 1;
        config.roi.y0 = yOffset + 1;
        config.roi.x1 = xOffset + imageWidth;
        config.roi.y1 = yOffset + imageHeight;
        config.trigger_mode = TRIGGER_MODE_EXTERNALTRIGGER;
        config.acquire_mode = ACQUIRE_MODE_AUTO;
        config.delay_time_s = 0;
        config.noise_filter_mode = NOISE_FILTER_MODE_ON;
        spdlog::info("Setting PCO camera configuration");
        camera.setConfiguration(config);
        spdlog::info("PCO camera configuration set");

        // Set exposure time
        spdlog::info("Setting PCO camera exposure time to {} us",
                     defaultExposureTimeUs);
        camera.setExposureTime(defaultExposureTimeUs / 1000000.0);
        camera.autoExposureOff();
        spdlog::info("PCO camera exposure time set");
    }

    int calculateOffset(int fullFrameSize, int roiSize)
    {
        return (fullFrameSize - roiSize) / 2;
    }

    void serveFrames(const std::string &shmFrameDataName,
                     const size_t frameBufferSize,
                     const std::string &shmExposureTimeName,
                     const std::string &shmFrameMetadataName,
                     const std::string &shmMutexName,
                     const std::string &shmCondVarName,
                     const unsigned int defaultExposureTimeUs,
                     const unsigned int imageWidth,
                     const unsigned int imageHeight,
                     const int xOffset,
                     const int yOffset,
                     const unsigned int fullFrameWidth,
                     const unsigned int fullFrameHeight)
    {
        // Setup shared memory buffers
        bool createNew = true;

        spdlog::info(
            "PCO camera server: Setting up shared memory buffers...");

        spdlog::info(
            "PCO camera server: Setting up shared memory for frame data");
        uint8_t *frameDataPtr;
        PCOSharedMemory::setupFrameData(
            shmFrameDataName, frameBufferSize, frameDataPtr, createNew);

        spdlog::info(
            "PCO camera server: Setting up shared memory for exposure time");
        unsigned int *exposureTimePtr;
        PCOSharedMemory::setupExposureTime(
            shmExposureTimeName, exposureTimePtr, createNew);

        spdlog::info(
            "PCO camera server: Setting up shared memory for frame metadata");
        PCOSharedMemory::FrameMetadata *frameMetadataPtr;
        PCOSharedMemory::setupFrameMetadata(
            shmFrameMetadataName, frameMetadataPtr, createNew);

        spdlog::info(
            "PCO camera server: Setting up shared memory for mutex");
        pthread_mutex_t *mutexPtr;
        PCOSharedMemory::setupMutex(shmMutexName, mutexPtr, createNew);

        spdlog::info(
            "PCO camera server: Setting up shared memory for cond var");
        pthread_cond_t *condVarPtr;
        PCOSharedMemory::setupConditionVariable(
            shmCondVarName, condVarPtr, createNew);

        spdlog::info("PCO camera server: Shared memory setup complete");

        // Set default exposure time and initial frame count
        spdlog::info(
            "Setting default exposure time in shared memory");
        *exposureTimePtr = defaultExposureTimeUs;

        // Initialize PCO camera
        spdlog::info("Setting up PCO camera");
        pco::Camera camera;
        PCOCameraServer::setupPCOCamera(camera,
                                        defaultExposureTimeUs,
                                        imageWidth,
                                        imageHeight,
                                        xOffset,
                                        yOffset,
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
        spdlog::info("Recording mode set to ring buffer with size {}",
                     bufferSize);

        unsigned int currentExposureTimeUs = defaultExposureTimeUs;

        // Data acquisition loop
        spdlog::info("PCO camera server starting its data acquisition loop");
        while (!PCOCameraServer::shutdownRequested.load())
        {
            // Check if we should change exposure time
            unsigned int setExposureTime = *exposureTimePtr;
            if (setExposureTime != currentExposureTimeUs)
            {
                spdlog::info(
                    "PCO camera server is changing exposure time to {} us",
                    setExposureTime);
                camera.setExposureTime(setExposureTime / 1000000.0);
                currentExposureTimeUs = setExposureTime;
                spdlog::info("Changed exposure time to {} us",
                             currentExposureTimeUs);
            }

            // Wait for new frame to arrive
            if (isFirstFrame)
            {
                camera.waitForFirstImage();
                isFirstFrame = false;
            }
            else
            {
                camera.waitForNewImage();
            }

            // Fetch image and convert to OpenCV format
            // spdlog::debug("PCO camera server got new frame. Serving.");
            camera.image(pcoImage,
                         PCO_RECORDER_LATEST_IMAGE,
                         pco::DataFormat::Mono16);
            cvImage = cv::Mat(pcoImage.height(),
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
            memcpy(frameMetadataPtr, &frameMetadata,
                   sizeof(PCOSharedMemory::FrameMetadata));
            pthread_cond_signal(condVarPtr);
            // spdlog::debug("PCO camera server signaled new frame");
            pthread_mutex_unlock(mutexPtr);
        }

        camera.stop();
        spdlog::info("PCO camera stopped.");
    }
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, PCOCameraServer::signalHandler);
    std::signal(SIGTERM, PCOCameraServer::signalHandler);

    PCOCameraServer::CLIOptions options =
        PCOCameraServer::parseCLI(argc, argv);
    spdlog::set_level(options.logLevel);

    std::filesystem::path profileDir =
        std::filesystem::path(PCOCameraServer::expandPath(options.profileDir));
    std::filesystem::path configPath = profileDir / "recorder_config.yaml";
    spdlog::info("Loading recorder configuration from {}",
                 configPath.string());
    RecorderConfig recorderConfig(configPath);

    const unsigned int fullFrameWidth =
        recorderConfig.getParameter<int>(
            "muscle_camera", "full_frame_width");
    const unsigned int fullFrameHeight =
        recorderConfig.getParameter<int>(
            "muscle_camera", "full_frame_height");
    const double defaultExposureTimeUs =
        recorderConfig.getParameter<unsigned int>(
            "muscle_camera", "default_exposure_time_us");

    // Validate image dimensions
    if (options.imageWidth <= 0 ||
        options.imageHeight <= 0 ||
        options.imageWidth > fullFrameWidth ||
        options.imageHeight > fullFrameHeight)
    {
        spdlog::critical(
            "Invalid image dimensions. "
            "Height must be within the range of 1 to {}; "
            "width must be within the range of 1 to {}",
            fullFrameHeight, fullFrameWidth);
        return 1;
    }

    if (options.xOffset >= (int)fullFrameWidth ||
        options.yOffset >= (int)fullFrameHeight ||
        options.xOffset < -1 ||
        options.yOffset < -1)
    {
        spdlog::critical(
            "Invalid offsets. "
            "X offset must be within the range of 0 to {}; "
            "Y offset must be within the range of 0 to {}. "
            "Alternatively, they can be set to -1, in which case the x and y "
            "offsets will be automatically calculated to put the ROI at the "
            "center of the sensor as much as possible.",
            fullFrameWidth - 1,
            fullFrameHeight - 1);
        return 1;
    }

    // Compute buffer size for each frame
    const size_t sizePerPixel = 2; // CV_16UC1
    const size_t frameBufferSize =
        options.imageWidth * options.imageHeight * sizePerPixel;

    // Set up shared memory buffers for frame data, mutex, and semaphore
    const std::string shmFrameDataName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_data_name");
    const std::string shmExposureTimeName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_exposure_time_name");
    const std::string shmFrameMetadataName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_metadata_name");
    const std::string shmMutexName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_mutex_name");
    const std::string shmCondVarName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_condition_variable_name");

    PCOCameraServer::serveFrames(shmFrameDataName,
                                 frameBufferSize,
                                 shmExposureTimeName,
                                 shmFrameMetadataName,
                                 shmMutexName,
                                 shmCondVarName,
                                 defaultExposureTimeUs,
                                 options.imageWidth,
                                 options.imageHeight,
                                 options.xOffset,
                                 options.yOffset,
                                 fullFrameWidth,
                                 fullFrameHeight);

    spdlog::info("PCO camera server stopping...");
    return 0;
}