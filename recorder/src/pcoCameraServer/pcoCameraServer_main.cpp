#include "pcoCameraServer.hpp"

namespace pcoCameraServer
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

    void setupPCOCamera(pco::Camera &camera,
                        unsigned int defaultExposureTimeUs,
                        unsigned int imageWidth,
                        unsigned int imageHeight,
                        unsigned int fullFrameWidth,
                        unsigned int fullFrameHeight)
    {
        int xOffset = calculateOffset(fullFrameWidth, imageWidth);
        int yOffset = calculateOffset(fullFrameHeight, imageHeight);

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

    void setupSharedMemory(const std::string &shmFrameDataName,
                           const size_t frameBufferSize,
                           const std::string &shmExposureTimeName,
                           const std::string &shmMutexName,
                           const std::string &shmFrameCountName,
                           uint8_t *&frameDataPtr,
                           unsigned int *&frameCountPtr,
                           unsigned int *&exposureTimePtr,
                           pthread_mutex_t *&mutex)
    {
        // Frame buffer
        spdlog::info("Setting up shared memory for PCO frame data");
        int shmFileDescFrameData = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescFrameData == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescFrameData, frameBufferSize) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        frameDataPtr = (uint8_t *)mmap(nullptr,
                                       frameBufferSize,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED,
                                       shmFileDescFrameData,
                                       0);
        if (frameDataPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescFrameData);

        // Frame count
        spdlog::info("Setting up shared memory for PCO frame count");
        int shmFileDescFrameCount = shm_open(
            shmFrameCountName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescFrameCount == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescFrameCount, sizeof(unsigned int)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        frameCountPtr = (unsigned int *)mmap(
            0,
            sizeof(unsigned int),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescFrameCount,
            0);
        if (frameCountPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescFrameCount);

        // Exposure time
        spdlog::info("Setting up shared memory for PCO exposure time");
        int shmFileDescExposureTime = shm_open(
            shmExposureTimeName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescExposureTime == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        size_t exposureTimeSize = sizeof(unsigned int);
        if (ftruncate(shmFileDescExposureTime, exposureTimeSize) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        exposureTimePtr = (unsigned int *)mmap(
            0,
            exposureTimeSize,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescExposureTime,
            0);
        if (exposureTimePtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescExposureTime);

        // Mutex
        spdlog::info("Setting up shared memory for PCO frame data mutex");
        int shmFileDescMutex = shm_open(
            shmMutexName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescMutex == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescMutex, sizeof(pthread_mutex_t)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        mutex = (pthread_mutex_t *)mmap(
            0,
            sizeof(pthread_mutex_t),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescMutex,
            0);
        if (mutex == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescMutex);
        // First-time init for mutex
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(mutex, &attr);
    }

    void serveFrames(const std::string &shmFrameDataName,
                     const size_t frameBufferSize,
                     const std::string &shmExposureTimeName,
                     const std::string &shmMutexName,
                     const std::string &shmFrameCountName,
                     const unsigned int defaultExposureTimeUs,
                     const unsigned int imageWidth,
                     const unsigned int imageHeight,
                     const unsigned int fullFrameWidth,
                     const unsigned int fullFrameHeight)
    {
        // Setup shared memory buffers
        spdlog::info("Setting up shared memory buffers for PCO camera");
        uint8_t *frameDataPtr;
        unsigned int *frameCountPtr;
        unsigned int *exposureTimePtr;
        pthread_mutex_t *mutex;
        setupSharedMemory(shmFrameDataName,
                          frameBufferSize,
                          shmExposureTimeName,
                          shmMutexName,
                          shmFrameCountName,
                          frameDataPtr,
                          frameCountPtr,
                          exposureTimePtr,
                          mutex);
        spdlog::info("Shared memory setup complete for PCO camera");

        // Set default exposure time and initial frame count
        spdlog::info(
            "Setting default exposure time and initial frame count in shared "
            "memory");
        *exposureTimePtr = defaultExposureTimeUs;
        *frameCountPtr = 0;

        // Initialize PCO camera
        spdlog::info("Setting up PCO camera");
        pco::Camera camera;
        pcoCameraServer::setupPCOCamera(camera,
                                        defaultExposureTimeUs,
                                        imageWidth,
                                        imageHeight,
                                        fullFrameWidth,
                                        fullFrameHeight);
        spdlog::info("PCO camera setup complete");

        // Create local data holders
        pco::Image pcoImage;
        cv::Mat cvImage;
        bool isFirstFrame = true;
        unsigned int frameId = 0;

        // Start camera acquisition
        spdlog::info("Starting PCO camera acquisition");
        int bufferSize = 10;
        camera.record(bufferSize, pco::RecordMode::ring_buffer);
        spdlog::info("Recording mode set to ring buffer with size {}",
                     bufferSize);

        unsigned int currentExposureTimeUs = defaultExposureTimeUs;

        // Data acquisition loop
        spdlog::info("PCO camera server starting its data acquisition loop");
        while (!pcoCameraServer::shutdownRequested.load())
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
            camera.image(pcoImage,
                         PCO_RECORDER_LATEST_IMAGE,
                         pco::DataFormat::Mono16);
            cvImage = cv::Mat(pcoImage.height(),
                              pcoImage.width(),
                              CV_16UC1,
                              pcoImage.raw_data().first);

            // Mutex-protected zone! Updata image buffer and frame count
            pthread_mutex_lock(mutex);
            memcpy(frameDataPtr, cvImage.data, frameBufferSize);
            *frameCountPtr = frameId++;
            pthread_mutex_unlock(mutex);
        }

        camera.stop();
        spdlog::info("PCO camera stopped.");
    }
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, pcoCameraServer::signalHandler);
    std::signal(SIGTERM, pcoCameraServer::signalHandler);

    pcoCameraServer::CLIOptions options =
        pcoCameraServer::parseCLI(argc, argv);
    spdlog::set_level(options.logLevel);

    std::filesystem::path profileDir =
        std::filesystem::path(pcoCameraServer::expandPath(options.profileDir));
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
        std::cerr
            << "Invalid image division. "
            << "Height must be within the range of 1 to " << fullFrameHeight
            << " and width must be within the range of 1 to " << fullFrameWidth
            << std::endl;
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
    const std::string shmMutexName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_mutex_name");
    const std::string shmExposureTimeName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_exposure_time_name");
    const std::string shmFrameCountName =
        recorderConfig.getParameter<std::string>(
            "muscle_camera", "shared_frame_count_name");

    pcoCameraServer::serveFrames(shmFrameDataName,
                                 frameBufferSize,
                                 shmExposureTimeName,
                                 shmMutexName,
                                 shmFrameCountName,
                                 defaultExposureTimeUs,
                                 options.imageWidth,
                                 options.imageHeight,
                                 fullFrameWidth,
                                 fullFrameHeight);

    spdlog::info("PCO camera server stopping...");
    return 0;
}