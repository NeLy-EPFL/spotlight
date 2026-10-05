#include "recorder/peripherals/muscle_camera.h"

#include <cerrno> // errno, ECHILD
#include <chrono>
#include <cstdlib> // std::abs
#include <filesystem>
#include <sys/mman.h>  // shm_unlink
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

// ---------------------------------------------------------------------------
// PcoCameraClient
// ---------------------------------------------------------------------------

PcoCameraClient::PcoCameraClient(
    pco_shared_memory::MuscleCameraRole role,
    unsigned int x0,
    unsigned int x1,
    unsigned int y0,
    unsigned int y1,
    const RecorderConfig &recorder_config)
    : role_name_(pco_shared_memory::role_to_string(role)),
      shm_names_(
          pco_shared_memory::get_shared_memory_names(recorder_config, role)),
      frame_buffer_size_((x1 - x0 + 1) * (y1 - y0 + 1) * 2), // CV_16UC1
      image_width_(x1 - x0 + 1), image_height_(y1 - y0 + 1) {
    // Remove regions left behind by a previous (possibly crashed) server, so
    // that we cannot attach to them and mistake their stale state (e.g.
    // is_ready) for the new server's. The new server recreates them.
    for (const std::string &name :
         {shm_names_.frame_data,
          shm_names_.shutter_open_time,
          shm_names_.server_state,
          shm_names_.mutex,
          shm_names_.condvar}) {
        shm_unlink(name.c_str()); // ENOENT (nothing left behind) is fine
    }
}

bool PcoCameraClient::is_ready() {
    // The server creates its shared memory right away but only reports ready
    // after the (slow) camera setup, once the camera is recording. Until then
    // the mutex and condvar may not be initialized, so nothing but is_ready
    // may be touched.
    if (server_state_ptr_ == nullptr) {
        try {
            attach_shared_memory();
        } catch (const std::runtime_error &e) {
            // Not (fully) created yet
            spdlog::debug("Shared memory not ready yet: {}", e.what());
            return false;
        }
    }
    return server_state_ptr_->is_ready.load();
}

void PcoCameraClient::attach_shared_memory() {
    // Map into locals and publish only once every region is mapped, so that a
    // partial failure leaves the client detached (server_state_ptr_ == nullptr)
    // and the caller can simply retry.
    bool create_new = false;
    uint8_t *frame_data_ptr;
    pco_shared_memory::ShutterOpenTime *shutter_open_time_ptr;
    pco_shared_memory::ServerState *server_state_ptr;
    pthread_mutex_t *mutex_ptr;
    pthread_cond_t *condvar_ptr;
    pco_shared_memory::setup_frame_data(
        shm_names_.frame_data, frame_buffer_size_, frame_data_ptr, create_new);
    pco_shared_memory::setup_shutter_open_time(
        shm_names_.shutter_open_time, shutter_open_time_ptr, create_new);
    pco_shared_memory::setup_mutex(shm_names_.mutex, mutex_ptr, create_new);
    pco_shared_memory::setup_condition_variable(
        shm_names_.condvar, condvar_ptr, create_new);
    // Last: its non-null pointer marks the client as attached.
    pco_shared_memory::setup_server_state(
        shm_names_.server_state, server_state_ptr, create_new);

    frame_data_ptr_ = frame_data_ptr;
    shutter_open_time_ptr_ = shutter_open_time_ptr;
    mutex_ptr_ = mutex_ptr;
    condvar_ptr_ = condvar_ptr;
    server_state_ptr_ = server_state_ptr;
    spdlog::info("Attached to shared memory of the {} camera", role_name_);
}

FrameData PcoCameraClient::wait_for_next_frame() {
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
    // real frames from 0 (see serve_frames), and last_frame_count_ starts at
    // -1, so this loop blocks until the first real frame instead of returning
    // the uninitialized buffer as a frame.
    while (server_state_ptr_->latest_frame.frame_count == last_frame_count_) {
        pthread_cond_wait(condvar_ptr_, mutex_ptr_);
    }

    pco_shared_memory::FrameMetadata metadata = server_state_ptr_->latest_frame;

    // Copy the frame out of shared memory while still holding the lock. The
    // cv::Mat below only wraps frame_data_ptr_, which the camera server
    // overwrites (memcpy) on every new frame; cloning under the lock takes a
    // private copy before the server can begin writing the next frame, so the
    // returned image can never be torn by a concurrent write.
    cv::Mat image =
        cv::Mat(image_height_, image_width_, CV_16UC1, frame_data_ptr_).clone();
    pthread_mutex_unlock(mutex_ptr_);

    // Detect lost frames. (last_frame_count_ == -1 is the initial state, before
    // any frame has been returned.)
    if (last_frame_count_ != -1) {
        // This is a single-slot handoff: the server memcpy's every frame into
        // the same buffer, so if we fell behind, frame_count has advanced by
        // more than one and the intervening frames are gone.
        if (metadata.frame_count != last_frame_count_ + 1) {
            spdlog::warn(
                "{} camera consumer fell behind: frame_count jumped from {} "
                "to {} ({} frame(s) dropped before they could be read).",
                role_name_,
                last_frame_count_,
                metadata.frame_count,
                metadata.frame_count - last_frame_count_ - 1);
        }
        // The server always fetches the latest image from the PCO recorder, so
        // if it fell behind the camera, the recorder image number skips.
        if (metadata.recorder_image_number != last_recorder_image_number_ + 1) {
            spdlog::warn(
                "{} camera server skipped camera images: recorder image number "
                "jumped from {} to {}.",
                role_name_,
                last_recorder_image_number_,
                metadata.recorder_image_number);
        }
    }

    if (image.empty()) {
        spdlog::error("{} camera client got an empty image", role_name_);
    }

    last_frame_count_ = metadata.frame_count;
    last_recorder_image_number_ = metadata.recorder_image_number;
    FrameData frame_data;
    frame_data.acquisition_time = metadata.acquisition_time;
    frame_data.received_time = get_current_time_microseconds();
    frame_data.image = image;
    frame_data.server_frame_count = metadata.frame_count;
    frame_data.recorder_image_number = metadata.recorder_image_number;
    return frame_data;
}

void PcoCameraClient::set_nominal_exposure_us(unsigned int exposure_us) {
    // The PCO camera server polls the requested value once per acquisition
    // loop iteration (at least every WAIT_TIMEOUT_SECS), applies it to the
    // camera, and echoes it back (see serve_frames() in
    // pco_camera_server_main.cc). Waiting for the echo lets callers order
    // later actions (e.g. re-syncing the trigger controller) after the change.
    constexpr auto timeout = std::chrono::seconds(2);
    constexpr auto poll_interval = std::chrono::milliseconds(5);
    shutter_open_time_ptr_->requested_us.store(exposure_us);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (shutter_open_time_ptr_->applied_us.load() != exposure_us) {
        if (std::chrono::steady_clock::now() > deadline) {
            spdlog::error(
                "PCO camera server for the {} camera did not apply exposure "
                "time {} us within {} s",
                role_name_,
                exposure_us,
                timeout.count());
            return;
        }
        std::this_thread::sleep_for(poll_interval);
    }
}

unsigned int PcoCameraClient::get_applied_exposure_us() const {
    return shutter_open_time_ptr_->applied_us.load();
}


// ---------------------------------------------------------------------------
// MuscleCamera
// ---------------------------------------------------------------------------

void MuscleCamera::start_server(
    const std::string &profile_dir, spdlog::level::level_enum log_level) {
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
        // ~MuscleCamera(), so the camera server is orphaned and keeps the
        // PCO camera open indefinitely. The next run then fails to open the
        // camera (it is already "attached") and the SDK reports the cryptic
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
    }

    // Parent process
    pid_ = pid;
    spdlog::info("PCO camera server started with PID {}", pid_);
}

void MuscleCamera::wait_until_ready(int timeout_s) {
    constexpr auto poll_interval = std::chrono::milliseconds(50);
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    while (!calcium_->is_ready() || !fiducial_->is_ready()) {
        if (waitpid(pid_, nullptr, WNOHANG) == pid_) {
            pid_ = -1; // already reaped
            throw std::runtime_error(
                "PCO camera server exited during startup (see its log above)");
        }
        if (std::chrono::steady_clock::now() > deadline) {
            stop();
            throw std::runtime_error(fmt::format(
                "PCO camera server was not ready within {} s", timeout_s));
        }
        std::this_thread::sleep_for(poll_interval);
    }
    spdlog::info("PCO camera server is ready");
}

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
      recorder_config_(recorder_config) {
    if (!is_roi_valid()) {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    // The clients remove stale shared memory, so create them before starting
    // the server. Both cameras are recording (waiting for acquire enable) once
    // this returns, so the trigger firmware's next STREAM starts them in sync.
    calcium_ = std::make_unique<PcoCameraClient>(
        pco_shared_memory::MuscleCameraRole::calcium,
        x0_,
        x1_,
        y0_,
        y1_,
        recorder_config);
    fiducial_ = std::make_unique<PcoCameraClient>(
        pco_shared_memory::MuscleCameraRole::fiducial,
        x0_,
        x1_,
        y0_,
        y1_,
        recorder_config);
    start_server(profile_dir, log_level);
    wait_until_ready(recorder_config.get_parameter<int>(
        "muscle_camera", "server_ready_timeout_s"));
}

MuscleCamera::~MuscleCamera() {
    stop();
}

void MuscleCamera::stop() {
    // Terminate the PCO camera server process. This is bounded: an unresponsive
    // server can never block shutdown indefinitely, because we escalate to
    // SIGKILL if it does not exit within the grace period. The process is
    // reaped in both paths so it does not linger as a zombie.
    //
    // Idempotent: pid_ is cleared once reaped, so a later call (e.g. an
    // explicit stop() followed by the destructor) is a no-op.
    if (pid_ <= 0) {
        return;
    }

    pid_t pid = pid_;
    pid_ = -1;

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

    spdlog::info("PCO camera server terminated.");
}


FramePair MuscleCamera::wait_for_next_frame_pair() {
    FrameData calcium = calcium_->wait_for_next_frame();
    FrameData fiducial = fiducial_->wait_for_next_frame();

    // Both servers stamp frames with the same host clock, so matching frames
    // were acquired within a fraction of a frame interval of each other. (The
    // cameras' own timestamps come from independent clocks and cannot be
    // compared.) Drop the older frame until the two match.
    const int64_t frame_interval_us =
        calcium_->get_applied_exposure_us() +
        static_cast<int64_t>(sensor_readout_time_us_);
    const int64_t tolerance_us = frame_interval_us / 2;
    while (true) {
        int64_t dt_us = static_cast<int64_t>(calcium.acquisition_time) -
                        static_cast<int64_t>(fiducial.acquisition_time);
        if (std::abs(dt_us) <= tolerance_us) {
            break;
        }
        if (dt_us > 0) {
            spdlog::warn(
                "Dropping unpaired fiducial camera frame (acquired {} us "
                "before the calcium frame)",
                dt_us);
            fiducial = fiducial_->wait_for_next_frame();
        } else {
            spdlog::warn(
                "Dropping unpaired calcium camera frame (acquired {} us before "
                "the fiducial frame)",
                -dt_us);
            calcium = calcium_->wait_for_next_frame();
        }
    }

    // Both cameras restart together on every sync, so between syncs the
    // difference of their recorder image numbers is constant. A change without
    // a preceding pause in acquisition (i.e. without a sync) means one camera
    // produced a frame the other did not: a drop or clock drift.
    long offset = static_cast<long>(fiducial.recorder_image_number) -
                  static_cast<long>(calcium.recorder_image_number);
    if (has_last_pair_ && offset != last_image_number_offset_) {
        bool after_pause = calcium.acquisition_time - last_pair_time_us_ >
                           static_cast<uint64_t>(frame_interval_us * 3 / 2);
        if (after_pause) {
            spdlog::info(
                "Muscle camera image number offset changed from {} to {} "
                "after a pause in acquisition (re-sync)",
                last_image_number_offset_,
                offset);
        } else {
            spdlog::warn(
                "Muscle camera image number offset changed from {} to {} "
                "without a re-sync (dropped frame or clock drift)",
                last_image_number_offset_,
                offset);
        }
    }
    has_last_pair_ = true;
    last_image_number_offset_ = offset;
    last_pair_time_us_ = calcium.acquisition_time;

    FramePair frame_pair;
    frame_pair.calcium = std::move(calcium);
    frame_pair.fiducial = std::move(fiducial);
    return frame_pair;
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
            full_frame_width,
            full_frame_height);
        return false;
    }

    return true;
}

void MuscleCamera::set_nominal_exposure_us(unsigned int exposure_us) {
    calcium_->set_nominal_exposure_us(exposure_us);
    fiducial_->set_nominal_exposure_us(exposure_us);
}

pid_t MuscleCamera::get_camera_server_pid() const {
    return pid_;
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
