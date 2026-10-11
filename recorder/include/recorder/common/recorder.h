#pragma once

#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/calibration.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/tracking_control.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"

#include <comm_protocol/protocol.h>

// Stage range (mm) covering the arena, for the motion-stage preview widget.
struct StageRange {
    double min_x_mm, max_x_mm, min_y_mm, max_y_mm;
};

// Parameters of one recording, chosen by the user (the GUI's spin boxes and
// protocol box, or run-spotlight-headless's parameters file). Whether muscle is
// imaged is not part of it: that is a live streaming setting of the Recorder
// (see Recorder::set_muscle_imaging_enabled()) that a recording inherits.
struct RecordingParams {
    int behavior_fps = 0;
    int sync_ratio = 1;
    int behavior_exposure_time_us = 0;
    int muscle_light_on_time_us = 0;
    // See parse_protocol_string(). Empty for an open recording.
    std::string experiment_protocol;
};

// The recording logic of run-spotlight, independent of any user interface.
//
// The constructor loads the recorder config, the arena registration model, and
// the muscle-camera ROIs, launches all acquisition, saving, tracking, and
// Arduino threads, and starts streaming (live preview, not saving). Recordings
// are then started and ended with start_recording() / end_recording(), and the
// program is ended with quit(). The GUI (MainGUIWindow) and
// run-spotlight-headless are thin front ends around this class.
//
// The worker threads hold references into this object, so it is neither
// copyable nor movable, and it must stay alive until quit().
class Recorder {
  public:
    // no_muscle: run without the muscle cameras, in which case muscle imaging
    //   is unavailable (muscle_camera() returns null).
    // Throws std::runtime_error if a required file is missing or malformed.
    Recorder(
        const std::filesystem::path &profile_dir,
        const std::filesystem::path &arena_dir,
        bool no_muscle);
    Recorder(const Recorder &) = delete;
    Recorder &operator=(const Recorder &) = delete;

    // The recording parameters from the recorder config's defaults.
    RecordingParams get_default_recording_params() const;

    // Toggle muscle imaging while streaming: whether the behavior camera is
    // locked to the muscle camera and the blue excitation LED is pulsed. Must
    // not be called during a recording (re-streaming would abort it).
    void set_muscle_imaging_enabled(bool enabled);
    bool is_muscle_imaging_enabled() const { return muscle_imaging_enabled_; }

    bool is_behavior_camera_ready() const;
    bool is_recording() const { return program_state_->is_recording.load(); }

    // Start a recording into save_directory(), which must not exist or be
    // empty (resolving that is up to the caller). Validates the parameters
    // before any side effect. Returns an error message if the recording could
    // not be started, std::nullopt on success.
    std::optional<std::string> start_recording(const RecordingParams &params);

    // End the recording and revert to streaming. reached_programmed_end is true
    // when a scheduled recording ran to its end (the controller has already
    // reverted on its own), false for a user-initiated stop.
    void end_recording(bool reached_programmed_end);

    // If a scheduled recording has reached its programmed end, finalize it via
    // end_recording(true) and return true. Meant to be polled periodically.
    bool finalize_programmed_stop_if_reached();

    // Block until all frames recorded so far have been handed to the saver
    // threads. The last few frames may still be being written when this
    // returns; quit() gives them ample time (it first stops the cameras, which
    // takes seconds) before telling the savers to stop.
    void wait_until_queues_drained() const;

    // Stop the cameras, the PCO server, the worker threads, and the excitation
    // light, then std::exit(). Some worker threads are intentionally parked in
    // blocking waits; they are reaped by the process exit rather than joined
    // (see the comments in the implementation).
    [[noreturn]] void quit(int exit_code = 0);

    // --- Accessors for front ends (previews, stage widget, save directory) ---
    const RecorderConfig &recorder_config() const { return recorder_config_; }
    const std::filesystem::path &profile_dir() const { return profile_dir_; }
    const std::shared_ptr<SaveDirectory> &save_directory() const {
        return save_directory_;
    }
    const std::shared_ptr<BehaviorRecordingState> &
    behavior_recording_state() const {
        return behavior_recording_state_;
    }
    const std::shared_ptr<MuscleRecordingState> &
    muscle_recording_state() const {
        return muscle_recording_state_;
    }
    const std::shared_ptr<TrackingControlState> &
    tracking_control_state() const {
        return tracking_control_state_;
    }
    // Null if started without the muscle cameras (no_muscle)
    const std::shared_ptr<MuscleCamera> &muscle_camera() const {
        return muscle_camera_;
    }
    const CalibrationParams &behavior_cam_calibration_params() const {
        return behavior_cam_calibration_params_;
    }
    const ActiveAreaMask &active_area_mask() const {
        return *active_area_mask_;
    }
    const StageRange &stage_range() const { return stage_range_; }
    int streaming_behavior_fps() const { return streaming_behavior_fps_; }
    int streaming_sync_ratio() const { return streaming_sync_ratio_; }

  private:
    // Assemble the STREAM / START_RECORDING params from the recording
    // parameters and the cached PCO timing.
    TriggerParams build_streaming_params() const;
    // musc_resync_interval: see get_musc_resync_interval(), computed once per
    // recording in start_recording().
    TriggerParams build_recording_params(
        const RecordingParams &params, unsigned int musc_resync_interval) const;
    // Program the free-running (auto-sequence) muscle camera's nominal exposure
    // so it produces muscle frames at beh_frame_rate / sync_ratio. Called at
    // startup and when a recording ends (streaming rate).
    void push_muscle_camera_exposure(int beh_frame_rate, int sync_ratio);

    // --- start_recording() helpers ---
    // Parse the experiment protocol, record the programmed-stop frame counts,
    // and (when imaging muscle) derive + validate the continuous-mode muscle
    // timing. Returns an error message if the protocol string or the muscle
    // timing is invalid; on success fills op_sequence and the derived muscle
    // timing (both 0 when muscle imaging is off).
    std::optional<std::string> validate_and_prepare_recording(
        const RecordingParams &params,
        std::deque<OperationStep> &op_sequence,
        int &muscle_nominal_exposure_us,
        int &muscle_buffer_time_us);
    // Write the recording metadata files into the (already created) save
    // directory. The muscle timing passed to write_experiment_parameters is the
    // derived continuous-mode timing from validate_and_prepare_recording() and
    // the number of muscle frames between periodic re-syncs.
    void write_experiment_parameters(
        const RecordingParams &params,
        int muscle_nominal_exposure_us,
        int muscle_buffer_time_us,
        unsigned int musc_resync_interval);
    void write_recorder_config();
    void write_behavior_calibration_parameters();
    // Only if the muscle cameras are running (not no_muscle)
    void write_muscle_camera_rois();
    void copy_homography_parameters_if_present();

    std::filesystem::path profile_dir_;
    RecorderConfig recorder_config_;
    CalibrationParams behavior_cam_calibration_params_;
    // Constructed once the calibration is known to be valid
    std::unique_ptr<ActiveAreaMask> active_area_mask_;
    StageRange stage_range_;

    std::shared_ptr<ProgramState> program_state_;
    std::shared_ptr<SaveDirectory> save_directory_;
    std::shared_ptr<BehaviorRecordingState> behavior_recording_state_;
    std::shared_ptr<MuscleRecordingState> muscle_recording_state_;
    std::shared_ptr<TrackingControlState> tracking_control_state_;
    std::shared_ptr<ProgrammedStop> programmed_recording_stop_;
    std::shared_ptr<MuscleCamera> muscle_camera_;
    std::shared_ptr<ArduinoCommunication> arduino_communication_;

    std::thread motion_control_io_thread_;
    std::thread motion_stage_position_logger_thread_;
    std::thread tracking_controller_thread_;
    std::thread behavior_image_acquirer_thread_;
    std::vector<std::thread> behavior_image_saver_threads_;
    std::thread muscle_image_acquirer_thread_;
    std::vector<std::thread> muscle_image_saver_threads_;

    int streaming_behavior_fps_ = 0;
    int streaming_sync_ratio_ = 1;
    // Default behavior exposure / muscle light-on times (us), from the recorder
    // config. Used for the streaming params the controller always runs; the
    // recording parameters take effect only when a recording starts (see
    // build_streaming_params).
    int default_beh_exp_time_us_ = 0;
    int default_musc_light_on_time_us_ = 0;
    bool muscle_imaging_enabled_ = false;

    // PCO sensor timing sent to the controller so it can derive the muscle
    // trigger delay (rolling time = scanned lines * line time).
    unsigned int pco_cam_rolling_time_us_ = 0;
    unsigned int pco_cam_readout_time_us_ = 0;
    // Muscle camera re-sync settings from the recorder config (see
    // get_musc_resync_interval() and TriggerParams::musc_acquire_restart_margin)
    unsigned int musc_resync_interval_s_ = 0;
    unsigned int musc_acquire_restart_margin_us_ = 0;
    // True while the in-progress recording is a scheduled one (non-empty
    // op_sequence). Controls how the recording is ended (see end_recording()).
    bool current_recording_is_scheduled_ = false;
};

// Parse the experiment-protocol string into an opSequence. Returns the number
// of steps, or -1 on a malformed string (see protocol_string_help).
int parse_protocol_string(
    const std::string &protocol_text_field_string,
    std::deque<OperationStep> &op_sequence);

// Description of the experiment-protocol string format, shown when it is
// malformed.
extern const char *const protocol_string_help;
