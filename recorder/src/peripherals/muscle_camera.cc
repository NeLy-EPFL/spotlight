#include "recorder/peripherals/muscle_camera.h"

#include <cerrno> // errno, ECHILD
#include <chrono>
#include <filesystem>
#include <sys/prctl.h> // prctl, PR_SET_PDEATHSIG
#include <thread>

namespace {
std::string log_level_to_str(spdlog::level::level_enum log_level) {
    switch (log_level) {
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
        spdlog::error("Unknown log level: {}. Using 'info'.", log_level);
        return "info";
    }
}
} // namespace

MuscleCamera::MuscleCamera(
    int image_width,
    int image_height,
    int x_offset,
    int y_offset,
    double rolling_shutter_line_time_us,
    double sensor_readout_time_us,
    const RecorderConfig &recorder_config,
    const std::string &profile_dir,
    spdlog::level::level_enum log_level)
    // Member initializers are in declaration order (avoids -Wreorder).
    : x0_(x_offset + 1), x1_(x_offset + image_width), y0_(y_offset + 1),
      y1_(y_offset + image_height), image_width_(image_width),
      image_height_(image_height),
      rolling_shutter_line_time_us_(rolling_shutter_line_time_us),
      sensor_readout_time_us_(sensor_readout_time_us),
      pco_camera_server_pid_(-1), frame_data_ptr_(nullptr),
      shutter_open_time_ptr_(nullptr), frame_metadata_ptr_(nullptr),
      mutex_ptr_(nullptr), cond_var_ptr_(nullptr),
      recorder_config_(recorder_config),
      // -1 = "no frame returned yet"; the server numbers real frames from 0.
      last_frame_count_(-1) {
    if (!is_roi_valid()) {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    // Capture our PID before forking so the child can detect (after arming its
    // parent-death signal below) whether we already died in the race window
    // between fork() and prctl().
    pid_t parent_pid_before_fork = getpid();

    pid_t pid = fork(); // DANGEROUS! Pay special attention to avoid fork bomb

    if (pid < 0) {
        std::string error_message =
            "Failed to fork process in order to start PCO camera server: " +
            std::string(strerror(errno));
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
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
        if (getppid() != parent_pid_before_fork) {
            _exit(EXIT_FAILURE);
        }

        // Resolve pco-camera-server alongside the running recorder binary so we
        // always launch the matching build, rather than whatever happens to be
        // on $PATH.
        std::filesystem::path server_path =
            std::filesystem::canonical("/proc/self/exe").parent_path() /
            "pco-camera-server";

        execl(
            server_path.c_str(),
            "pco-camera-server",
            "--profile-dir",
            profile_dir.c_str(),
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
            log_level_to_str(log_level).c_str(),
            (char *)nullptr);

        // If execl returns, it must have failed
        std::string error_message = "Failed to execute PCO camera server at " +
                                    server_path.string() + ": " +
                                    std::string(strerror(errno));
        spdlog::critical(error_message);
        exit(EXIT_FAILURE); // Exit child process
    } else {
        // Parent process
        pco_camera_server_pid_ = pid;
        spdlog::info(
            "PCO camera server started with process ID (PID): {}",
            pco_camera_server_pid_);

        // Wait for the camera server to initialize
        sleep(1); // sleep for 1 second

        // Setup shared memory buffers
        spdlog::info("Muscle camera API: Setting up shared memory buffer...");

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame data");
        bool create_new = false;

        size_t frame_buffer_size = image_width * image_height * 2; // CV_16UC1
        std::string shm_frame_data_name =
            recorder_config.get_parameter<std::string>(
                "muscle_camera", "shared_frame_data_name");
        pco_shared_memory::setup_frame_data(
            shm_frame_data_name,
            frame_buffer_size,
            frame_data_ptr_,
            create_new);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for shutter-open "
            "time");
        std::string shm_shutter_open_time_name =
            recorder_config.get_parameter<std::string>(
                "muscle_camera", "shared_shutter_open_time_name");
        pco_shared_memory::setup_shutter_open_time(
            shm_shutter_open_time_name, shutter_open_time_ptr_, create_new);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for frame metadata");
        std::string shm_frame_metadata_name =
            recorder_config.get_parameter<std::string>(
                "muscle_camera", "shared_frame_metadata_name");
        pco_shared_memory::setup_frame_metadata(
            shm_frame_metadata_name, frame_metadata_ptr_, create_new);

        spdlog::info("Muscle camera API: Setting up shared memory for mutex");
        std::string shm_mutex_name = recorder_config.get_parameter<std::string>(
            "muscle_camera", "shared_mutex_name");
        pco_shared_memory::setup_mutex(shm_mutex_name, mutex_ptr_, create_new);

        spdlog::info(
            "Muscle camera API: Setting up shared memory for cond var");
        std::string shm_cond_var_name =
            recorder_config.get_parameter<std::string>(
                "muscle_camera", "shared_condition_variable_name");
        pco_shared_memory::setup_condition_variable(
            shm_cond_var_name, cond_var_ptr_, create_new);
        spdlog::info("Shared memory setup complete for PCO camera");
    }
}

void MuscleCamera::stop() {
    // Terminate the PCO camera server process. This is bounded: an unresponsive
    // server can never block shutdown indefinitely, because we escalate to
    // SIGKILL if it does not exit within the grace period. The process is
    // reaped in both paths so it does not linger as a zombie.
    //
    // Idempotent: pcoCameraServerPID_ is cleared once reaped, so a later call
    // (e.g. an explicit stop() followed by the destructor) is a no-op.
    if (pco_camera_server_pid_ <= 0) {
        return;
    }

    pid_t pid = pco_camera_server_pid_;
    pco_camera_server_pid_ = -1;

    kill(pid, SIGTERM);

    // Poll for graceful exit up to a bounded deadline before escalating. The
    // server checks its shutdown flag once per frame-wait timeout (0.1 s), so
    // it normally exits well within this window.
    constexpr int grace_period_ms = 3000;
    constexpr int poll_interval_ms = 20;
    bool reaped = false;
    for (int elapsed_ms = 0; elapsed_ms < grace_period_ms;
         elapsed_ms += poll_interval_ms) {
        pid_t result = waitpid(pid, nullptr, WNOHANG);
        if (result == pid || (result == -1 && errno == ECHILD)) {
            reaped = true;
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(poll_interval_ms));
    }

    if (!reaped) {
        spdlog::warn(
            "PCO camera server (PID {}) did not exit within {} ms of SIGTERM; "
            "escalating to SIGKILL.",
            pid,
            grace_period_ms);
        kill(pid, SIGKILL);
        // SIGKILL cannot be caught or ignored, so this blocking reap is
        // bounded.
        waitpid(pid, nullptr, 0);
    }

    spdlog::info("PCO camera server process terminated.");
}

MuscleCamera::~MuscleCamera() {
    stop();
}

FrameData MuscleCamera::wait_for_one_frame() {
    pthread_mutex_lock(mutex_ptr_);

    // Wait until a frame newer than the last one we returned is published.
    // Looping on this predicate (rather than waiting unconditionally) is what
    // makes the handoff correct: a spurious wakeup simply re-waits, and -- more
    // importantly -- a signal delivered by the server in the window between our
    // previous unlock and this wait is never lost, because the predicate
    // already reflects the bumped frame_count. POSIX condition variables do not
    // latch, so without this check that signal would be missed and we would
    // block until the *next* frame.
    //
    // The server writes frame_count = -1 before producing anything and numbers
    // real frames from 0 (see serve_frames), and last_frame_count_ starts at -1,
    // so this loop blocks until the first real frame instead of returning the
    // uninitialized buffer as a frame.
    while (frame_metadata_ptr_->frame_count == last_frame_count_) {
        pthread_cond_wait(cond_var_ptr_, mutex_ptr_);
    }

    long frame_count = frame_metadata_ptr_->frame_count;
    uint64_t acquisition_time = frame_metadata_ptr_->acquisition_time;

    // Detect frames that were overwritten before we could read them. This is a
    // single-slot handoff: the server memcpy's every frame into the same buffer,
    // so if we fell behind, frame_count has advanced by more than one and the
    // intervening frames are gone. Warn rather than fail -- the acquirer
    // renumbers frames contiguously, so silent drops would otherwise misalign
    // the muscle and behavior frame streams in a recording. (last == -1 is the
    // initial state, before any frame has been returned.)
    if (last_frame_count_ != -1 && frame_count != last_frame_count_ + 1) {
        spdlog::warn(
            "Muscle camera consumer fell behind: frame_count jumped from {} to "
            "{} ({} frame(s) dropped before they could be read).",
            last_frame_count_,
            frame_count,
            frame_count - last_frame_count_ - 1);
    }

    // Copy the frame out of shared memory while still holding the lock. The
    // cv::Mat below only wraps frame_data_ptr_, which the camera server
    // overwrites (memcpy) on every new frame; cloning under the lock takes a
    // private copy before the server can begin writing the next frame, so the
    // returned image can never be torn by a concurrent write.
    cv::Mat image =
        cv::Mat(image_height_, image_width_, CV_16UC1, frame_data_ptr_).clone();
    pthread_mutex_unlock(mutex_ptr_);

    if (image.empty()) {
        spdlog::error("muscle_camera API got an empty image");
    }

    last_frame_count_ = frame_count;
    FrameData frame_data;
    frame_data.acquisition_time = acquisition_time;
    frame_data.received_time = get_current_time_microseconds();
    frame_data.image = image;
    return frame_data;
}

bool MuscleCamera::is_roi_valid() const {
    int full_frame_width = recorder_config_.get_parameter<int>(
        "muscle_camera", "full_frame_width");
    int full_frame_height = recorder_config_.get_parameter<int>(
        "muscle_camera", "full_frame_height");
    if (x0_ < 1 || x1_ > full_frame_width || y0_ < 1 ||
        y1_ > full_frame_height || x0_ >= x1_ || y0_ >= y1_ ||
        image_width_ % 32 != 0 || image_height_ % 8 != 0 || image_width_ < 64 ||
        image_height_ < 16) {
        spdlog::critical(
            "Invalid ROI for muscle camera. The following conditions must be "
            "met: 1 <= x0 < x1 <= {}; 1 <= y0 < y1 <= {}. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels. The width must be a "
            "multiple of 32 and the height must be a multiple of 8.",
            image_width_,
            image_height_);
        return false;
    }

    return true;
}

void MuscleCamera::set_nominal_exposure_us(unsigned int exposure_us) {
    if (shutter_open_time_ptr_ != nullptr) {
        // The PCO camera server polls this shared value in its acquisition loop
        // and applies it as the camera's nominal per-line exposure (see
        // serve_frames() in pco_camera_server_main.cc). In continuous mode this
        // also sets the free-run frame rate.
        *shutter_open_time_ptr_ = exposure_us;
    } else {
        spdlog::error(
            "Cannot set exposure time. Shared memory pointer is null.");
    }
}

pid_t MuscleCamera::get_camera_server_pid() const {
    return pco_camera_server_pid_;
}

int MuscleCamera::get_num_lines_scanned() const {
    return image_height_;
}

int round_to_nearest_valid_muscle_cam_horizontal(int value) {
    int remainder = value % 32;
    return value - remainder + (remainder < 16 ? 0 : 32);
}

int round_to_nearest_valid_muscle_cam_vertical(int value) {
    int remainder = value % 8;
    return value - remainder + (remainder < 4 ? 0 : 8);
}

bool MuscleTriggerTiming::compute_parameters(
    int muscle_image_height,
    double muscle_camera_line_scan_time_us,
    int muscle_camera_readout_time_us) {
    double muscle_camera_fps = behavior_camera_fps_ / double(sync_ratio_);
    int muscle_interval_us = 1000000 / muscle_camera_fps;
    int rolling_time_us = muscle_image_height * muscle_camera_line_scan_time_us;

    // Continuous rolling shutter (auto-sequence): the camera free-runs at
    // 1/(nominal_exposure + readout). Set the nominal per-line exposure so one
    // frame fills the requested muscle interval, then split it into the rolling
    // time (line skew) and the common time (all lines exposing simultaneously).
    // The light-on window must fit inside the common time, leaving a
    // non-negative buffer:
    //   nominal_exposure = muscle_interval - readout = rolling_time +
    //   common_time common_time      = light_on + buffer_time
    // Valid only if rolling_time + light_on + readout <= muscle_interval. (Note
    // the single rolling_time: triggered acquisition would need 2 *
    // rolling_time, but continuous rolling has no idle line-reset time -- see
    // docs/data_acquisition.md.)
    nominal_exposure_us_ = muscle_interval_us - muscle_camera_readout_time_us;
    common_time_us_ = nominal_exposure_us_ - rolling_time_us;
    buffer_time_us_ = common_time_us_ - muscle_light_on_time_us_;
    if (buffer_time_us_ < 0) {
        spdlog::critical(
            "Computed muscle camera parameters are invalid: "
            "rolling_time + light_on + readout must be <= muscle_interval. "
            "rolling_time_us = {}, "
            "muscle_camera_readout_time_us = {}, "
            "muscle_light_on_time_us = {}, "
            "muscle_interval_us = {}",
            rolling_time_us,
            muscle_camera_readout_time_us,
            muscle_light_on_time_us_,
            muscle_interval_us);
        return false; // Invalid configuration
    }
    return true;
}
