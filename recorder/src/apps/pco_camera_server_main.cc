#include "recorder/apps/pco_camera_server.h"

namespace pco_camera_server {
void print_help(const char *program_name) {
    // clang-format off
    std::cout
        << "Usage: " << program_name << " [OPTIONS]\n"
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

spdlog::level::level_enum parse_log_level(const std::string &level) {
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

CLIOptions parse_cli(int argc, char **argv) {
    CLIOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            std::exit(0);
        } else if (arg == "-v" || arg == "--verbose") {
            options.log_level = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            options.log_level = parse_log_level(argv[++i]);
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc) {
            options.profile_dir = argv[++i];
        } else if ((arg == "-x0" || arg == "--x-min") && i + 1 < argc) {
            options.x0 = std::stoi(argv[++i]);
        } else if ((arg == "-x1" || arg == "--x-max") && i + 1 < argc) {
            options.x1 = std::stoi(argv[++i]);
        } else if ((arg == "-y0" || arg == "--y-min") && i + 1 < argc) {
            options.y0 = std::stoi(argv[++i]);
        } else if ((arg == "-y1" || arg == "--y-max") && i + 1 < argc) {
            options.y1 = std::stoi(argv[++i]);
        } else if ((arg == "-d" || arg == "--delay") && i + 1 < argc) {
            options.delay_us = std::stoi(argv[++i]);
        } else if (arg[0] == '-') {
            spdlog::critical("Unknown option: {}", arg);
            print_help(argv[0]);
            std::exit(1);
        } else if (i == 1 && arg[0] != '-') {
            // Support for positional argument (for backward compatibility)
            options.profile_dir = arg;
        } else {
            spdlog::critical("Unknown option: {}", arg);
            print_help(argv[0]);
            std::exit(1);
        }
    }

    return options;
}

std::string expand_path(const std::string &path) {
    // Check if the path starts with "~/"
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        // Get the HOME environment variable
        const char *home_dir = std::getenv("HOME");

        // If HOME is available, replace "~/" with the home directory
        if (home_dir) {
            std::filesystem::path expanded_path =
                std::filesystem::path(home_dir) / path.substr(2);
            return expanded_path.string();
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
void signal_handler(int signal) {
    const char *signal_name = signal == SIGINT    ? "SIGINT"
                              : signal == SIGTERM ? "SIGTERM"
                                                  : "Unknown signal";
    spdlog::info(
        "Shutdown signal received ({}: {}). Cleaning up and exiting...",
        signal,
        signal_name);
    shutdown_requested.store(true);
}

uint64_t get_current_time_microseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

// Decode a single packed-BCD byte (two decimal digits) to its numeric value.
static uint32_t decode_bcd_byte(uint8_t byte) {
    return (byte >> 4) * 10 + (byte & 0x0F);
}

// Read the PCO camera's own frame timestamp, in microseconds since midnight.
// The value comes from the per-image PCO metadata (pco::Image::getMetaDataPtr(),
// populated by camera.image() when metadata mode is on -- see setup_pco_camera).
// Unlike the binary-timestamp feature, metadata is delivered as a separate block
// and does NOT overwrite any image pixels. Unlike get_current_time_microseconds
// (a host clock) this is the camera's own clock, so it is unaffected by
// host-side scheduling/handoff latency. It is a time-of-day, so it wraps at
// midnight -- fine for the intra-recording relative timing downstream relies on.
uint64_t get_camera_timestamp_microseconds(pco::Image &image) {
    const PCO_METADATA_STRUCT *metadata = image.getMetaDataPtr();
    if (metadata == nullptr) {
        spdlog::error(
            "PCO image has no metadata; is metadata mode enabled on the "
            "camera?");
        return 0;
    }
    // bIMAGE_TIME_US_BCD is 6 BCD digits across 3 bytes, least-significant byte
    // first (see PCO_METADATA_STRUCT in sc2_common.h).
    uint32_t microseconds = decode_bcd_byte(metadata->bIMAGE_TIME_US_BCD[0]) +
                            decode_bcd_byte(metadata->bIMAGE_TIME_US_BCD[1]) *
                                100 +
                            decode_bcd_byte(metadata->bIMAGE_TIME_US_BCD[2]) *
                                10000;
    uint32_t seconds = decode_bcd_byte(metadata->bIMAGE_TIME_SEC_BCD);
    uint32_t minutes = decode_bcd_byte(metadata->bIMAGE_TIME_MIN_BCD);
    uint32_t hours = decode_bcd_byte(metadata->bIMAGE_TIME_HOUR_BCD);
    return static_cast<uint64_t>(hours) * 3600000000ULL +
           static_cast<uint64_t>(minutes) * 60000000ULL +
           static_cast<uint64_t>(seconds) * 1000000ULL + microseconds;
}

void setup_pco_camera(
    pco::Camera &camera,
    unsigned int default_shutter_open_time_us,
    unsigned int x0,
    unsigned int x1,
    unsigned int y0,
    unsigned int y1,
    unsigned int delay_us,
    unsigned int full_frame_width,
    unsigned int full_frame_height) {
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
    // instead of waiting for an external TTL trigger per frame. This is
    // required by the acquisition design (docs/data_acquisition.md): the
    // trigger firmware does NOT trigger this camera -- it locks the behavior
    // camera to the muscle camera's free-running common-time signal on SMA #4
    // (configured below). With TRIGGER_MODE_EXTERNALTRIGGER the camera would
    // wait forever for a trigger the firmware never sends, never expose, never
    // drive SMA #4, and the firmware would in turn wait forever for the
    // common-time onset -- freezing both cameras. The free-run frame rate is
    // set via the nominal exposure (see setExposureTime below and the
    // acquisition loop's live exposure updates).
    config.trigger_mode = TRIGGER_MODE_AUTOTRIGGER;
    config.acquire_mode = ACQUIRE_MODE_AUTO;
    // Zero inter-frame delay keeps the rolling shutter continuous (no idle
    // time). Any sync delay is implemented in the trigger firmware, not here.
    config.delay_time_s = delay_us / 1000000.0; // Convert to seconds
    config.noise_filter_mode = NOISE_FILTER_MODE_ON;
    // Enable per-image metadata so every frame carries the camera's own
    // timestamp and image counter (read back via pco::Image::getMetaDataPtr();
    // see get_camera_timestamp_microseconds). Metadata is delivered as a
    // separate block and, unlike the binary-timestamp feature
    // (TIMESTAMP_MODE_BINARY), does NOT overwrite any image pixels.
    config.metadata_mode = METADATA_MODE_ON;
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
        delay_us);

    // Set exposure time
    spdlog::info(
        "Setting PCO camera shutter-open time to {} us",
        default_shutter_open_time_us);
    camera.setExposureTime(default_shutter_open_time_us / 1000000.0);
    camera.autoExposureOff();
    spdlog::info("PCO camera shutter-open time set");

    // Set trigger polarity
    spdlog::info("Setting PCO camera trigger polarity to rising edge");
    camera.configureHWIO_1_exposureTrigger(
        true, pco::HWIO_EdgePolarity::rising_edge);

    // Drive SMA #4 as the muscle camera's "common time" reference for the
    // trigger firmware. The firmware (trigger_firmware:
    // DeviceIO::isMuscCommonTime) treats the line being HIGH as "in common
    // time" and fires the behavior frame
    // + blue LED on the LOW->HIGH onset, so the camera must drive the line HIGH
    // for exactly the common-time window.
    //
    // - signal_type status_expos: report the exposure status on SMA #4.
    // - timing global: for a rolling shutter, "global" is the interval when all
    //   lines are exposed simultaneously, i.e. the common time (see
    //   docs/data_acquisition.md). NOT all_lines, which spans the whole rolling
    //   exposure envelope and would make the onset fire ~rolling_time too
    //   early.
    // - polarity high_level: status_expos is asserted during the global window,
    // so
    //   high_level makes the line HIGH during common time (LOW otherwise),
    //   matching the firmware's edge.
    camera.configureHWIO_4_statusExpos(
        true,
        pco::HWIO_Polarity::high_level,
        pco::HWIO_4_SignalType::status_expos,
        pco::HWIO_StatusExpos_Timing::global); // global = common time
}

void serve_frames(
    const std::string &shm_frame_data_name,
    const size_t frame_buffer_size,
    const std::string &shm_shutter_open_time_name,
    const std::string &shm_frame_metadata_name,
    const std::string &shm_mutex_name,
    const std::string &shm_cond_var_name,
    const unsigned int default_shutter_open_time_us,
    const unsigned int x0,
    const unsigned int x1,
    const unsigned int y0,
    const unsigned int y1,
    const unsigned int delay_us,
    const unsigned int full_frame_width,
    const unsigned int full_frame_height) {
    // Setup shared memory buffers
    bool create_new = true;

    spdlog::info("PCO camera server: Setting up shared memory buffers...");

    spdlog::info("PCO camera server: Setting up shared memory for frame data");
    uint8_t *frame_data_ptr;
    pco_shared_memory::setup_frame_data(
        shm_frame_data_name, frame_buffer_size, frame_data_ptr, create_new);

    spdlog::info(
        "PCO camera server: Setting up shared memory for exposure time");
    unsigned int *shutter_open_time_ptr;
    pco_shared_memory::setup_shutter_open_time(
        shm_shutter_open_time_name, shutter_open_time_ptr, create_new);

    spdlog::info(
        "PCO camera server: Setting up shared memory for frame metadata");
    pco_shared_memory::FrameMetadata *frame_metadata_ptr;
    pco_shared_memory::setup_frame_metadata(
        shm_frame_metadata_name, frame_metadata_ptr, create_new);

    spdlog::info("PCO camera server: Setting up shared memory for mutex");
    pthread_mutex_t *mutex_ptr;
    pco_shared_memory::setup_mutex(shm_mutex_name, mutex_ptr, create_new);

    spdlog::info("PCO camera server: Setting up shared memory for cond var");
    pthread_cond_t *cond_var_ptr;
    pco_shared_memory::setup_condition_variable(
        shm_cond_var_name, cond_var_ptr, create_new);

    spdlog::info("PCO camera server: Shared memory setup complete");

    // Set default exposure time and initial frame count
    spdlog::info("Setting default exposure time in shared memory");
    *shutter_open_time_ptr = default_shutter_open_time_us;
    // Mark "no frame published yet" before the (slow) camera setup below, so
    // the consumer (MuscleCamera::wait_for_one_frame) never mistakes the
    // zero-filled buffer for frame 0. Done here, right after the region is
    // created, so it is in place well before the consumer maps it. Real frames
    // are numbered from 0.
    frame_metadata_ptr->frame_count = -1;

    // Initialize PCO camera
    spdlog::info("Setting up PCO camera");
    pco::Camera camera;
    pco_camera_server::setup_pco_camera(
        camera,
        default_shutter_open_time_us,
        x0,
        x1,
        y0,
        y1,
        delay_us,
        full_frame_width,
        full_frame_height);
    spdlog::info("PCO camera setup complete");

    // Create local data holders
    pco::Image pco_image;
    cv::Mat cv_image;
    bool is_first_frame = true;
    long frame_count = 0;

    // Start camera acquisition
    spdlog::info("Starting PCO camera acquisition");
    int buffer_size = 10;
    camera.record(buffer_size, pco::RecordMode::ring_buffer);
    spdlog::info("Recording mode set to ring buffer with size {}", buffer_size);

    unsigned int current_exposure_time_us = default_shutter_open_time_us;

    // Data acquisition loop
    spdlog::info("PCO camera server starting its data acquisition loop");
    while (!shutdown_requested.load()) {
        // Check if we should change exposure time
        unsigned int target_exposure_time = *shutter_open_time_ptr;
        if (target_exposure_time != current_exposure_time_us) {
            spdlog::info(
                "PCO camera server is changing exposure time to {} us",
                target_exposure_time);
            camera.setExposureTime(target_exposure_time / 1000000.0);
            current_exposure_time_us = target_exposure_time;
            spdlog::info(
                "Changed exposure time to {} us", current_exposure_time_us);
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
        // a chance to check if shutdown_requested has been set to true and
        // break accordingly.
        // spdlog::debug("Entering waiting inner loop");
        while (true) {
            if (is_first_frame) {
                try {
                    camera.waitForFirstImage(
                        WAIT_WITH_SMALL_DELAY, WAIT_TIMEOUT_SECS);
                    is_first_frame = false;
                    // spdlog::debug(
                    //     "First frame received, breaking out of waiting "
                    //     "inner loop");
                    break;
                } catch (pco::CameraException &e) {
                    uint32_t error_code = e.error_code();
                    // spdlog::debug(
                    //     "Exception while waiting for first image; "
                    //     "error code 0x{0:08x}",
                    //     error_code);
                    if (error_code == TIMEOUT_ERROR_CODE) {
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
                    uint32_t error_code = e.error_code();
                    // spdlog::debug(
                    //     "Exception while waiting for first image; "
                    //     "error code 0x{0:08x}",
                    //     error_code);
                    if (error_code == TIMEOUT_ERROR_CODE) {
                        // This is expected, so do nothing
                    } else {
                        throw;
                    }
                }
            }
            if (shutdown_requested.load()) {
                spdlog::info(
                    "PCO camera server: Shutdown requested, breaking out "
                    "of inner waiting loop");
                break;
            }
        }
        if (shutdown_requested.load()) {
            break;
        }

        // Fetch image and convert to OpenCV format
        // spdlog::debug("PCO camera server got new frame. Serving.");
        camera.image(
            pco_image, PCO_RECORDER_LATEST_IMAGE, pco::DataFormat::Mono16);
        cv_image = cv::Mat(
            pco_image.height(),
            pco_image.width(),
            CV_16UC1,
            pco_image.raw_data().first);

        // Gather metadata. acquisition_time is the PCO camera's own frame
        // timestamp (from per-image metadata, not a host clock); pco_record_id
        // is the camera recorder's running image number, useful for spotting
        // dropped/duplicated frames.
        pco_shared_memory::FrameMetadata frame_metadata;
        frame_metadata.frame_count = frame_count++;
        frame_metadata.acquisition_time =
            pco_camera_server::get_camera_timestamp_microseconds(pco_image);
        frame_metadata.pco_record_id = pco_image.getRecorderImageNumber();

        // Mutex-protected zone! Updata image buffer and frame count
        pthread_mutex_lock(mutex_ptr);
        memcpy(frame_data_ptr, cv_image.data, frame_buffer_size);
        memcpy(
            frame_metadata_ptr,
            &frame_metadata,
            sizeof(pco_shared_memory::FrameMetadata));
        pthread_cond_signal(cond_var_ptr);
        // spdlog::debug("PCO camera server signaled new frame");
        pthread_mutex_unlock(mutex_ptr);
    }

    camera.stop();
    spdlog::info("PCO camera stopped.");
}
} // namespace pco_camera_server

int main(int argc, char *argv[]) {
    std::signal(SIGINT, pco_camera_server::signal_handler);
    std::signal(SIGTERM, pco_camera_server::signal_handler);

    pco_camera_server::CLIOptions options =
        pco_camera_server::parse_cli(argc, argv);
    spdlog::set_level(options.log_level);

    std::filesystem::path profile_dir = std::filesystem::path(
        pco_camera_server::expand_path(options.profile_dir));
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "pco-camera-server loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);

    const unsigned int full_frame_width =
        recorder_config.get_parameter<int>("muscle_camera", "full_frame_width");
    const unsigned int full_frame_height = recorder_config.get_parameter<int>(
        "muscle_camera", "full_frame_height");
    unsigned int roi_width = options.x1 - options.x0 + 1;
    unsigned int roi_height = options.y1 - options.y0 + 1;

    // Initial nominal per-line exposure for the free-running (auto-sequence)
    // camera. In continuous mode the exposure sets the frame rate
    // (rate = 1/(exposure + readout)), so derive it from the default streaming
    // muscle interval (streaming sync ratio / streaming behavior FPS):
    //   exposure = muscle_interval - readout.
    // The recorder GUI overwrites this live (via the shared shutter-open-time
    // region) as soon as it knows the active streaming/recording parameters.
    const double sensor_readout_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    const int streaming_beh_fps = recorder_config.get_parameter<int>(
        "behavior_camera", "streaming_frame_rate");
    const int streaming_sync_ratio = recorder_config.get_parameter<int>(
        "muscle_camera", "streaming_sync_ratio");
    const unsigned int default_muscle_interval_us = static_cast<unsigned int>(
        1000000.0 * streaming_sync_ratio / streaming_beh_fps);
    const unsigned int default_shutter_open_time_us =
        default_muscle_interval_us -
        static_cast<unsigned int>(sensor_readout_time_us);

    // Validate image dimensions
    if (options.x0 == 0 || options.y0 == 0 || options.x1 > full_frame_width ||
        options.y1 > full_frame_height || options.x0 >= options.x1 ||
        options.y0 >= options.y1) {
        spdlog::critical(
            "Invalid image dimensions. The following is required: "
            "0 < x0 < x1 <= {}; 0 < y0 < y1 <= {}.",
            full_frame_width,
            full_frame_height);
        return 1;
    }

    if (roi_width % 32 != 0 || roi_height % 8 != 0 || roi_width < 64 ||
        roi_height < 16) {
        spdlog::critical(
            "Invalid ROI for muscle camera. ROI width must be a multiple of "
            "32 and ROI height must be a multiple of 8. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels.");
        return 1;
    }

    // Compute buffer size for each frame
    const size_t size_per_pixel = 2; // CV_16UC1
    const size_t frame_buffer_size = roi_width * roi_height * size_per_pixel;

    // Set up shared memory buffers for frame data, mutex, and semaphore
    const std::string shm_frame_data_name =
        recorder_config.get_parameter<std::string>(
            "muscle_camera", "shared_frame_data_name");
    const std::string shm_shutter_open_time_name =
        recorder_config.get_parameter<std::string>(
            "muscle_camera", "shared_shutter_open_time_name");
    const std::string shm_frame_metadata_name =
        recorder_config.get_parameter<std::string>(
            "muscle_camera", "shared_frame_metadata_name");
    const std::string shm_mutex_name =
        recorder_config.get_parameter<std::string>(
            "muscle_camera", "shared_mutex_name");
    const std::string shm_cond_var_name =
        recorder_config.get_parameter<std::string>(
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
        pco_camera_server::serve_frames(
            shm_frame_data_name,
            frame_buffer_size,
            shm_shutter_open_time_name,
            shm_frame_metadata_name,
            shm_mutex_name,
            shm_cond_var_name,
            default_shutter_open_time_us,
            options.x0,
            options.x1,
            options.y0,
            options.y1,
            options.delay_us,
            full_frame_width,
            full_frame_height);
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