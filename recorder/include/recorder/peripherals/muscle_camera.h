#pragma once

#include <cstdlib> // exit
#include <iostream>
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

class MuscleCamera {
  public:
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
    // Terminate the PCO camera server process. Bounded (SIGTERM, then SIGKILL
    // after a grace period) so an unresponsive server can never block shutdown
    // indefinitely. Idempotent and safe to call before destruction.
    void stop();
    FrameData wait_for_one_frame();
    // Program the camera's nominal per-line exposure (us). In continuous
    // (auto-sequence) mode this also sets the free-run frame rate, since the
    // camera runs at 1/(nominal_exposure + readout). Derive the value with
    // MuscleTriggerTiming::get_nominal_exposure_us().
    void set_nominal_exposure_us(unsigned int exposure_us);
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
    pid_t pco_camera_server_pid_;
    uint8_t *frame_data_ptr_;
    unsigned int *shutter_open_time_ptr_;
    pco_shared_memory::FrameMetadata *frame_metadata_ptr_;
    pthread_mutex_t *mutex_ptr_;
    pthread_cond_t *cond_var_ptr_;
    const RecorderConfig &recorder_config_;
    unsigned int last_frame_count_;

    bool is_roi_valid() const;
};

int round_to_nearest_valid_muscle_cam_horizontal(int value);
int round_to_nearest_valid_muscle_cam_vertical(int value);
