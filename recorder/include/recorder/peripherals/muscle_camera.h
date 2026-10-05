#pragma once

#include <array>
#include <cstdlib> // exit
#include <iostream>
#include <memory>
#include <signal.h> // kill, SIGINT
#include <string>
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <unistd.h>    // fork, exec

#include <spdlog/spdlog.h>

#include "recorder/apps/shared_memory_utils.h"
#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

// Derives the muscle camera's continuous-mode (auto-sequence) exposure from the
// recording parameters and validates that the requested muscle frame interval
// is long enough to fit the rolling shutter, light-on, and sensor readout
// times.
//
// In continuous mode the camera free-runs at 1/(nominal_exposure + readout), so
// the nominal per-line exposure programmed into the camera is set so that one
// frame fills the desired muscle frame interval:
//   nominal_exposure = muscle_interval - readout = rolling_time + common_time
//   common_time      = light_on + buffer_time   (the SMA#4 common-time HIGH
//   window)
// The blue excitation LED is pulsed for light_on within the common time, which
// the trigger firmware locks to via the SMA#4 status line. See
// docs/data_acquisition.md.
class MuscleTriggerTiming {
  public:
    MuscleTriggerTiming(
        int behavior_camera_fps, int sync_ratio, int muscle_light_on_time_us)
        : behavior_camera_fps_(behavior_camera_fps), sync_ratio_(sync_ratio),
          muscle_light_on_time_us_(muscle_light_on_time_us) {}

    // Populates the derived getters below. Returns false if the configuration
    // is invalid (the muscle frame interval is too short to fit the light-on
    // window inside the common time, i.e. the buffer time would be negative).
    bool compute_parameters(
        int muscle_image_height,
        double muscle_camera_line_scan_time_us,
        int muscle_camera_readout_time_us);

    // Nominal per-line exposure to program into the camera (us).
    int get_nominal_exposure_us() const {
        return nominal_exposure_us_;
    }
    // Slack in the common-time window beyond the light-on time (us).
    int get_buffer_time_us() const {
        return buffer_time_us_;
    }
    // Duration of the common time / SMA#4 HIGH window (us).
    int get_common_time_us() const {
        return common_time_us_;
    }

  private:
    // User input parameters
    int behavior_camera_fps_;
    int sync_ratio_;
    int muscle_light_on_time_us_;

    // Derived parameters
    int nominal_exposure_us_ = -1;
    int buffer_time_us_ = -1;
    int common_time_us_ = -1;
};

// Client of one camera served by pco-camera-server: attaches to the camera's
// shared memory and reads its frames.
class PcoCameraClient {
  public:
    // Remove the camera's shared memory left behind by a previous server (call
    // before starting the server). The ROI is 1-indexed and inclusive.
    PcoCameraClient(
        pco_shared_memory::MuscleCameraRole role,
        unsigned int x0,
        unsigned int x1,
        unsigned int y0,
        unsigned int y1,
        const RecorderConfig &recorder_config);
    PcoCameraClient(const PcoCameraClient &) = delete;
    PcoCameraClient &operator=(const PcoCameraClient &) = delete;

    // Attach to the camera's shared memory if not done yet, and return true
    // once the server reports that the camera is recording.
    bool is_ready();
    // Block until the server publishes a frame newer than the last one
    // returned, and return a copy of it.
    FrameData wait_for_next_frame();
    // Program the camera's nominal per-line exposure (us) and block until the
    // server has applied it (logs an error on timeout).
    void set_nominal_exposure_us(unsigned int exposure_us);
    // The nominal exposure (us) the server last applied to the camera.
    unsigned int get_applied_exposure_us() const;

  private:
    // Map all of the server's shared-memory regions (throws if any does not
    // exist or is not sized yet).
    void attach_shared_memory();

    std::string role_name_; // "calcium" or "fiducial"
    pco_shared_memory::SharedMemoryNames shm_names_;
    size_t frame_buffer_size_;
    unsigned int image_width_;
    unsigned int image_height_;
    uint8_t *frame_data_ptr_ = nullptr;
    pco_shared_memory::ShutterOpenTime *shutter_open_time_ptr_ = nullptr;
    pco_shared_memory::ServerState *server_state_ptr_ = nullptr;
    pthread_mutex_t *mutex_ptr_ = nullptr;
    pthread_cond_t *condvar_ptr_ = nullptr;
    // Index of the last frame returned, or -1 before any has been returned
    // (matches the server's FrameMetadata::frame_count sentinel).
    long last_frame_count_ = -1;
    uint32_t last_recorder_image_number_ = 0;
};

// The two PCO muscle cameras (calcium and fiducial), with the same ROI, served
// by one pco-camera-server process (the PCO SDK does not support opening them
// from separate processes). The trigger firmware starts
// them together via their shared acquire-enable line; after that each runs on
// its own internal clock. Frames are paired by host acquisition time.
class MuscleCamera {
  public:
    // Start the camera server and block until both cameras are recording.
    // Throws if the server exits or is not ready within
    // muscle_camera/server_ready_timeout_s. The ROI is given as size and
    // 0-indexed offset.
    MuscleCamera(
        int image_width,
        int image_height,
        int x_offset,
        int y_offset,
        double rolling_shutter_line_time_us,
        double sensor_readout_time_us,
        const RecorderConfig &recorder_config,
        const std::string &profile_dir,
        spdlog::level::level_enum log_level);
    ~MuscleCamera();
    MuscleCamera(const MuscleCamera &) = delete;
    MuscleCamera &operator=(const MuscleCamera &) = delete;
    // Terminate the camera server. Bounded (SIGTERM, then SIGKILL after a grace
    // period) so an unresponsive server can never block shutdown indefinitely.
    // Idempotent and safe to call before destruction.
    void stop();
    // Block until both cameras have a new frame acquired within half a frame
    // interval of each other, and return them. Unpaired frames are dropped
    // with a warning.
    FramePair wait_for_next_frame_pair();
    // Program both cameras' nominal per-line exposure (us) and block until both
    // servers have applied it. In continuous (auto-sequence) mode this also
    // sets the free-run frame rate, since the cameras run at
    // 1/(nominal_exposure + readout). Derive the value with
    // MuscleTriggerTiming::get_nominal_exposure_us(). The cameras switch at
    // slightly different times, so re-sync them afterwards (the trigger
    // firmware does so on every STREAM and START_RECORDING).
    void set_nominal_exposure_us(unsigned int exposure_us);
    // PID of the camera server (-1 once stopped).
    pid_t get_camera_server_pid() const;
    int get_num_lines_scanned() const;

  private:
    unsigned int x0_;
    unsigned int x1_;
    unsigned int y0_;
    unsigned int y1_;
    unsigned int image_width_;
    unsigned int image_height_;
    double rolling_shutter_line_time_us_;
    double sensor_readout_time_us_;
    const RecorderConfig &recorder_config_;
    std::unique_ptr<PcoCameraClient> calcium_;
    std::unique_ptr<PcoCameraClient> fiducial_;
    pid_t pid_ = -1; // camera server
    // State for the cross-camera consistency check in
    // wait_for_next_frame_pair(). The recorder image numbers of paired frames
    // differ by a constant between syncs.
    bool has_last_pair_ = false;
    long last_image_number_offset_ = 0;
    uint64_t last_pair_time_us_ = 0;

    bool is_roi_valid() const;
    // Fork and exec pco-camera-server.
    void start_server(
        const std::string &profile_dir, spdlog::level::level_enum log_level);
    // Poll until the server reports both cameras ready.
    void wait_until_ready(int timeout_s);
};

int round_to_nearest_valid_muscle_cam_horizontal(int value);
int round_to_nearest_valid_muscle_cam_vertical(int value);
