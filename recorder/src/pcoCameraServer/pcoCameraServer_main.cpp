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
        std::cout << "Setting up camera" << std::endl;
        std::cout << "Getting default configuration" << std::endl;
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
        std::cout << "Setting configuration" << std::endl;
        camera.setConfiguration(config);
        std::cout << "Configuration set" << std::endl;

        // Set exposure time
        std::cout << "Setting exposure time" << std::endl;
        camera.setExposureTime(defaultExposureTimeUs / 1000000.0);
        camera.autoExposureOff();
        std::cout << "Exposure time set" << std::endl;
    }

    int calculateOffset(int fullFrameSize, int roiSize)
    {
        return (fullFrameSize - roiSize) / 2;
    }

    void setupSharedMemory(const std::string &shmFrameDataName,
                           const size_t frameBufferSize,
                           const std::string &shmExposureTimeName,
                           const std::string &shmMutexName,
                           uint8_t *&frameDataPtr,
                           unsigned int *&frameCountPtr,
                           unsigned int *&exposureTimePtr,
                           pthread_mutex_t *&mutex)
    {
        // Frame buffer
        int shmFileDescFrameData = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_WRONLY, 0666);
        ftruncate(shmFileDescFrameData, frameBufferSize);
        frameDataPtr = (uint8_t *)mmap(0,
                                       frameBufferSize,
                                       PROT_WRITE,
                                       MAP_SHARED,
                                       shmFileDescFrameData,
                                       0);

        // Frame count
        int shmFileDescFrameCount = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_WRONLY, 0666);
        ftruncate(shmFileDescFrameCount, sizeof(unsigned int));
        frameCountPtr = (unsigned int *)mmap(
            0,
            sizeof(unsigned int),
            PROT_WRITE,
            MAP_SHARED,
            shmFileDescFrameCount,
            0);

        // Exposure time
        int shmFileDescExposureTime = shm_open(
            shmExposureTimeName.c_str(), O_CREAT | O_RDWR, 0666);
        size_t exposureTimeSize = sizeof(unsigned int);
        ftruncate(shmFileDescExposureTime, exposureTimeSize);
        exposureTimePtr = (unsigned int *)mmap(
            0,
            exposureTimeSize,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescExposureTime,
            0);

        // Mutex
        int shmFileDescMutex = shm_open(
            shmMutexName.c_str(), O_CREAT | O_RDWR, 0666);
        ftruncate(shmFileDescMutex, sizeof(pthread_mutex_t));
        mutex = (pthread_mutex_t *)mmap(
            0,
            sizeof(pthread_mutex_t),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescMutex,
            0);
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
                     const unsigned int defaultExposureTimeUs,
                     const unsigned int imageWidth,
                     const unsigned int imageHeight,
                     const unsigned int fullFrameWidth,
                     const unsigned int fullFrameHeight)
    {
        // Setup shared memory buffers
        uint8_t *frameDataPtr;
        unsigned int *frameCountPtr;
        unsigned int *exposureTimePtr;
        pthread_mutex_t *mutex;
        setupSharedMemory(shmFrameDataName,
                          frameBufferSize,
                          shmExposureTimeName,
                          shmMutexName,
                          frameDataPtr,
                          frameCountPtr,
                          exposureTimePtr,
                          mutex);
        // Set default exposure time
        *exposureTimePtr = defaultExposureTimeUs;

        // Initialize PCO camera
        pco::Camera camera;
        pcoCameraServer::setupPCOCamera(camera,
                                        defaultExposureTimeUs,
                                        imageWidth,
                                        imageHeight,
                                        fullFrameWidth,
                                        fullFrameHeight);

        // Create local data holders
        pco::Image pcoImage;
        cv::Mat cvImage;
        cv::Mat displayImage;
        bool isFirstFrame = true;
        unsigned int frameId = 0;
        int isListenerReady;

        // Setup fd set for checking if we should write to FIFO
        fd_set write_fds;
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10; // 10 microseconds

        // Start camera acquisition
        int bufferSize = 10;
        camera.record(bufferSize, pco::RecordMode::ring_buffer);
        spdlog::info("Recording mode set to ring buffer with size {}",
                     bufferSize);

        unsigned int currentExposureTimeUs = defaultExposureTimeUs;

        while (!pcoCameraServer::shutdownRequested.load())
        {
            // Check if we should change exposure time
            unsigned int setExposureTime = *exposureTimePtr;
            if (setExposureTime != currentExposureTimeUs)
            {
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
                                 defaultExposureTimeUs,
                                 options.imageWidth,
                                 options.imageHeight,
                                 fullFrameWidth,
                                 fullFrameHeight);

    spdlog::info("PCO camera server stopping...");
    return 0;
}