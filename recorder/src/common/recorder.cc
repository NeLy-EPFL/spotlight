#include "recorder/common/recorder.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "recorder/common/stage_timing.h"

namespace {

RecorderConfig load_recorder_config(const std::filesystem::path &profile_dir) {
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "Loading recorder configuration from {}", config_path.string());
    RecorderConfig recorder_config(config_path);
    if (!recorder_config.is_defined) {
        std::string error_message = fmt::format(
            "Failed to load recorder configuration. Cannot start recording. "
            "Expected valid recorder configuration file at {}",
            config_path.c_str());
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    return recorder_config;
}

std::filesystem::path
get_calibration_params_file_path(const std::filesystem::path &arena_dir) {
    return arena_dir / "model" / "calibration_result.yaml";
}

// Compute the stage range covering the arena, for the motion-stage preview
// widget. Read arena dimensions from <arena_dir>/metadata.yaml, then invert the
// calibration model at the image center to find which stage position
// corresponds to each of the four arena corners. The computed limits are
// clipped to [0, physical_range_limit_mm].
//
// Side effect: registers the clipped limits as software motion stage limits via
// set_motion_stage_limits() so set_target_motion_stage_position() clamps.
StageRange compute_stage_range_from_arena(
    const std::filesystem::path &arena_dir,
    const RecorderConfig &recorder_config,
    const CalibrationParams &behavior_cam_calibration_params) {
    std::filesystem::path arena_metadata_path = arena_dir / "metadata.yaml";
    if (!std::filesystem::exists(arena_metadata_path)) {
        std::string error_message = fmt::format(
            "Arena metadata file not found: {}", arena_metadata_path.string());
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    YAML::Node arena_metadata = YAML::LoadFile(arena_metadata_path.string());
    auto arena_dim = arena_metadata["arena_dim"].as<std::vector<double>>();
    double arena_size_x_mm = arena_dim[0];
    double arena_size_y_mm = arena_dim[1];
    // Saved/registration images are rotated 90 deg CCW from the raw
    // sensor, so the image's column count equals the sensor ROI height
    // and its row count equals the ROI width.
    int roi_width =
        recorder_config.get_parameter<int>("behavior_camera", "roi_width");
    int roi_height =
        recorder_config.get_parameter<int>("behavior_camera", "roi_height");
    int image_center_col = roi_height / 2;
    int image_center_row = roi_width / 2;
    double stage_min_x_mm = std::numeric_limits<double>::infinity();
    double stage_max_x_mm = -std::numeric_limits<double>::infinity();
    double stage_min_y_mm = std::numeric_limits<double>::infinity();
    double stage_max_y_mm = -std::numeric_limits<double>::infinity();
    for (const auto &corner : std::vector<std::pair<double, double>>{
             {0.0, 0.0},
             {arena_size_x_mm, 0.0},
             {arena_size_x_mm, arena_size_y_mm},
             {0.0, arena_size_y_mm},
         }) {
        auto [sx, sy] = behavior_cam_calibration_params
                            .physical_pos_and_pixel_pos_to_stage_pos(
                                corner.first,
                                corner.second,
                                image_center_row,
                                image_center_col);
        stage_min_x_mm = std::min(stage_min_x_mm, sx);
        stage_max_x_mm = std::max(stage_max_x_mm, sx);
        stage_min_y_mm = std::min(stage_min_y_mm, sy);
        stage_max_y_mm = std::max(stage_max_y_mm, sy);
    }
    spdlog::info(
        "Stage range covering arena {}x{} mm: X=[{:.3f}, {:.3f}], "
        "Y=[{:.3f}, {:.3f}] mm",
        arena_size_x_mm,
        arena_size_y_mm,
        stage_min_x_mm,
        stage_max_x_mm,
        stage_min_y_mm,
        stage_max_y_mm);

    // Clip computed limits to [0, physical_range_limit_mm] and register them
    // as software motion stage limits so set_target_motion_stage_position()
    // clamps.
    double physical_range_limit_mm = recorder_config.get_parameter<double>(
        "motion_control", "physical_range_limit_mm");
    auto clip_to_physical_range =
        [physical_range_limit_mm](double val, const char *name) -> double {
        double clipped = std::clamp(val, 0.0, physical_range_limit_mm);
        if (clipped != val) {
            spdlog::warn(
                "Software stage limit {} ({:.3f} mm) falls outside physical "
                "range [0, {:.3f}] mm; clamping to {:.3f} mm.",
                name,
                val,
                physical_range_limit_mm,
                clipped);
        }
        return clipped;
    };
    stage_min_x_mm = clip_to_physical_range(stage_min_x_mm, "stage_min_x");
    stage_max_x_mm = clip_to_physical_range(stage_max_x_mm, "stage_max_x");
    stage_min_y_mm = clip_to_physical_range(stage_min_y_mm, "stage_min_y");
    stage_max_y_mm = clip_to_physical_range(stage_max_y_mm, "stage_max_y");
    set_motion_stage_limits(
        stage_min_x_mm, stage_max_x_mm, stage_min_y_mm, stage_max_y_mm);

    return {stage_min_x_mm, stage_max_x_mm, stage_min_y_mm, stage_max_y_mm};
}

} // namespace

Recorder::Recorder(
    const std::filesystem::path &profile_dir,
    const std::filesystem::path &arena_dir,
    bool no_muscle)
    : profile_dir_(profile_dir),
      recorder_config_(load_recorder_config(profile_dir)),
      behavior_cam_calibration_params_(
          get_calibration_params_file_path(arena_dir).string()),
      program_state_(std::make_shared<ProgramState>()),
      behavior_recording_state_(std::make_shared<BehaviorRecordingState>()),
      muscle_recording_state_(std::make_shared<MuscleRecordingState>()),
      tracking_control_state_(std::make_shared<TrackingControlState>()),
      programmed_recording_stop_(std::make_shared<ProgrammedStop>()) {
    // Atomic holder of the save directory
    save_directory_ = std::make_shared<SaveDirectory>(
        recorder_config_.get_parameter<std::string>("gui", "default_save_dir"));

    // Position mapping / calibration parameters from the arena registration
    // scan (`fit-arena-registration` output).
    if (!behavior_cam_calibration_params_.is_defined) {
        std::string error_message = fmt::format(
            "Spatial calibration data not found or malformed. This is required "
            "for tracking and recording. Expected valid calibration file at {} "
            "(produced by `fit-arena-registration -a <arena_dir>`).",
            get_calibration_params_file_path(arena_dir).string());
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    spdlog::info(
        "Loaded spatial calibration parameters from {}",
        get_calibration_params_file_path(arena_dir).string());

    // Active-area mask for closed-loop tracking
    double boundary_margin_mm = recorder_config_.get_parameter<double>(
        "tracking", "boundary_margin_mm");
    active_area_mask_ = std::make_unique<ActiveAreaMask>(
        arena_dir.string(),
        boundary_margin_mm,
        behavior_cam_calibration_params_.stage_and_pixel_to_physical);
    spdlog::info("Loaded active area mask from {}", arena_dir.string());

    // Also registers software motion stage limits as a side effect.
    stage_range_ = compute_stage_range_from_arena(
        arena_dir, recorder_config_, behavior_cam_calibration_params_);

    behavior_recording_state_->latest_frame_holder =
        std::make_shared<LatestFrame>();

    // Start tracking & motion control threads
    motion_control_io_thread_ = std::thread(
        motion_control_request_handler,
        recorder_config_,
        tracking_control_state_,
        program_state_);
    motion_stage_position_logger_thread_ = std::thread(
        motion_stage_position_logger,
        recorder_config_,
        tracking_control_state_,
        save_directory_,
        program_state_);
    tracking_controller_thread_ = std::thread(
        tracking_controller,
        recorder_config_,
        std::ref(*active_area_mask_),
        behavior_recording_state_,
        tracking_control_state_,
        std::ref(behavior_cam_calibration_params_),
        program_state_);

    // Start behavior image acquirer and savers
    behavior_image_acquirer_thread_ = std::thread(
        behavior_image_acquirer,
        recorder_config_,
        behavior_recording_state_,
        program_state_,
        programmed_recording_stop_);
    spdlog::info("Behavior camera acquisition thread started");
    int num_behavior_image_saver_threads = recorder_config_.get_parameter<int>(
        "behavior_camera", "num_image_saving_threads");
    for (int i = 0; i < num_behavior_image_saver_threads; i++) {
        behavior_image_saver_threads_.push_back(std::thread(
            behavior_image_saver,
            recorder_config_,
            behavior_recording_state_,
            save_directory_,
            program_state_));
    }
    spdlog::info("Behavior camera saver threads started");

    // Start the muscle image acquirer and savers, unless running without the
    // muscle cameras. muscle_camera_ then stays null, which tells the front
    // ends that muscle imaging is unavailable.
    if (no_muscle) {
        spdlog::info("Running without the muscle cameras (--no-muscle)");
    } else {
        MuscleCameraROIs muscle_rois =
            get_muscle_camera_rois(profile_dir_ / "muscle_camera_roi.yaml");
        muscle_image_acquirer_thread_ = std::thread(
            muscle_image_acquirer,
            muscle_rois,
            recorder_config_,
            profile_dir_,
            spdlog::get_level(),
            muscle_recording_state_,
            program_state_,
            programmed_recording_stop_);
        spdlog::info("Muscle camera acquisition thread started");
        size_t retry_count = 0;
        while (!muscle_recording_state_->muscle_camera.load()) {
            // Wait for the muscle camera to be initialized
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            spdlog::debug("Waiting for muscle camera to be initialized...");
            retry_count++;
            if (retry_count % 10 == 0) {
                spdlog::warn("Muscle camera is not initialized.");
            }
        }
        muscle_camera_ = muscle_recording_state_->muscle_camera.load();

        int num_muscle_image_saver_threads =
            recorder_config_.get_parameter<int>(
                "muscle_camera", "num_image_saving_threads");
        for (int i = 0; i < num_muscle_image_saver_threads; i++) {
            muscle_image_saver_threads_.push_back(std::thread(
                muscle_image_saver,
                recorder_config_,
                muscle_recording_state_,
                save_directory_,
                program_state_,
                5)); // cv::IMWRITE_TIFF_COMPRESSION_LZW
        }
        spdlog::info("Muscle camera saver threads started");
    }

    // Streaming parameters and PCO sensor timing, sent in every STREAM /
    // START_RECORDING. The controller derives the muscle trigger delay from
    // the PCO timing (the rolling time is the time to scan all lines of the
    // muscle ROI). Without the muscle cameras, muscle imaging stays disabled
    // and the rolling time is unused.
    streaming_behavior_fps_ = recorder_config_.get_parameter<int>(
        "behavior_camera", "streaming_frame_rate");
    streaming_sync_ratio_ = recorder_config_.get_parameter<int>(
        "muscle_camera", "streaming_sync_ratio");
    default_beh_exp_time_us_ = recorder_config_.get_parameter<int>(
        "behavior_camera", "default_exposure_time_us");
    default_musc_light_on_time_us_ = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_light_on_time_us");
    double rolling_shutter_line_time_us =
        recorder_config_.get_parameter<double>(
            "muscle_camera", "rolling_shutter_line_time_us");
    pco_cam_rolling_time_us_ =
        muscle_camera_ ? static_cast<unsigned int>(
                             muscle_camera_->get_num_lines_scanned() *
                             rolling_shutter_line_time_us)
                       : 0;
    pco_cam_readout_time_us_ =
        static_cast<unsigned int>(recorder_config_.get_parameter<double>(
            "muscle_camera", "sensor_readout_time_us"));
    musc_resync_interval_s_ = recorder_config_.get_parameter<unsigned int>(
        "muscle_camera", "resync_interval_s");
    musc_acquire_restart_margin_us_ =
        recorder_config_.get_parameter<unsigned int>(
            "muscle_camera", "acquire_restart_margin_us");

    // Initialize the free-running (auto-sequence) muscle camera to the
    // streaming muscle frame rate. In continuous mode the nominal exposure sets
    // the frame rate; it is switched to the recording rate when a recording
    // starts and back when it ends (see start_recording/end_recording).
    push_muscle_camera_exposure(streaming_behavior_fps_, streaming_sync_ratio_);

    // Start Arduino triggering interface
    std::string arduino_port_name = find_arduino_port_name(recorder_config_);
    arduino_communication_ =
        std::make_shared<ArduinoCommunication>(arduino_port_name);

    // Reboot the trigger controller into a clean, known state at startup. The
    // comm thread waits for the reboot and reopens the port, so the first
    // STREAM below reaches the freshly reset controller.
    arduino_communication_->reset();

    // Start in streaming mode: live preview only, not saving. Muscle imaging is
    // off by default (enable_muscle = false: behavior camera free-runs, blue
    // excitation LED off). The controller is configured with a single STREAM
    // command.
    arduino_communication_->stream(build_streaming_params());
}

RecordingParams Recorder::get_default_recording_params() const {
    RecordingParams params;
    params.behavior_fps = recorder_config_.get_parameter<int>(
        "behavior_camera", "default_recording_fps");
    params.sync_ratio = recorder_config_.get_parameter<int>(
        "muscle_camera", "default_recording_sync_ratio");
    params.behavior_exposure_time_us = default_beh_exp_time_us_;
    params.muscle_light_on_time_us = default_musc_light_on_time_us_;
    return params;
}

void Recorder::set_muscle_imaging_enabled(bool enabled) {
    if (enabled && !muscle_camera_) {
        spdlog::error("Cannot enable muscle imaging without muscle cameras");
        return;
    }
    if (program_state_->is_recording.load()) {
        spdlog::error("Cannot toggle muscle imaging during a recording");
        return;
    }
    muscle_imaging_enabled_ = enabled;
    spdlog::info(
        enabled ? "Enabling muscle imaging" : "Disabling muscle imaging");
    // Re-stream so the controller switches modes immediately
    arduino_communication_->stream(build_streaming_params());
}

bool Recorder::is_behavior_camera_ready() const {
    std::shared_ptr<BehaviorCamera> behavior_camera =
        behavior_recording_state_->behavior_camera.load();
    return behavior_camera && behavior_camera->is_ready();
}

std::optional<std::string>
Recorder::validate_and_prepare_recording(
    const RecordingParams &params,
    std::deque<OperationStep> &op_sequence,
    int &muscle_nominal_exposure_us,
    int &muscle_buffer_time_us) {
    muscle_nominal_exposure_us = 0;
    muscle_buffer_time_us = 0;

    // Parse the experiment protocol into an opSequence and record the
    // programmed-stop frame counts. A malformed string aborts the recording.
    int num_steps_parsed =
        parse_protocol_string(params.experiment_protocol, op_sequence);
    spdlog::info("Parsed {} protocol steps", op_sequence.size());
    if (num_steps_parsed < 0) {
        return std::string("Invalid experiment protocol.\n\n") +
               protocol_string_help;
    } else if (num_steps_parsed == 0) {
        spdlog::info("Starting recording without any protocol steps");
        programmed_recording_stop_->num_behavior_frames_expected = -1;
        programmed_recording_stop_->num_muscle_frames_expected = -1;
    } else {
        spdlog::info(
            "Starting recording with {} protocol steps", op_sequence.size());
        programmed_recording_stop_->num_behavior_frames_expected =
            op_sequence.back().frame_idx;
        programmed_recording_stop_->num_muscle_frames_expected =
            op_sequence.back().frame_idx / params.sync_ratio;
        spdlog::info(
            "Setting expected number of steps to {} (behavior) and {} (muscle)",
            programmed_recording_stop_->num_behavior_frames_expected,
            programmed_recording_stop_->num_muscle_frames_expected);
    }
    // An empty opSequence is an open recording; a non-empty one is scheduled.
    current_recording_is_scheduled_ = num_steps_parsed > 0;

    // When imaging muscle, derive and validate the continuous-mode muscle
    // trigger timing from the recording parameters.
    if (muscle_imaging_enabled_) {
        MuscleTriggerTiming muscle_trigger_timing(
            params.behavior_fps,
            params.sync_ratio,
            params.muscle_light_on_time_us);
        double rolling_shutter_line_time_us =
            recorder_config_.get_parameter<double>(
                "muscle_camera", "rolling_shutter_line_time_us");
        if (!muscle_trigger_timing.compute_parameters(
                muscle_camera_->get_num_lines_scanned(),
                rolling_shutter_line_time_us,
                pco_cam_readout_time_us_)) {
            return std::string(
                "Invalid muscle recording configuration. In particular, check "
                "that the muscle recording interval (1 / muscle FPS) is long "
                "enough for the rolling shutter, light-on, and sensor readout "
                "times. See "
                "https://github.com/NeLy-EPFL/spotlight-control/issues/79.");
        }
        muscle_nominal_exposure_us =
            muscle_trigger_timing.get_nominal_exposure_us();
        muscle_buffer_time_us = muscle_trigger_timing.get_buffer_time_us();
    }

    return std::nullopt;
}

std::optional<std::string>
Recorder::start_recording(const RecordingParams &params) {
    if (!is_behavior_camera_ready()) {
        spdlog::error("Behavior camera not ready. Cannot start recording.");
        return std::string("Behavior camera not ready.");
    }

    // Validate the experiment protocol and (when imaging muscle) the muscle
    // trigger timing before any side effects -- camera exposure change,
    // directory creation, metadata writes -- so an invalid configuration
    // aborts cleanly and leaves nothing behind. The derived muscle timing is
    // returned for the metadata and the exposure update below.
    std::deque<OperationStep> op_sequence;
    int muscle_nominal_exposure_us = 0;
    int muscle_buffer_time_us = 0;
    if (std::optional<std::string> error = validate_and_prepare_recording(
            params,
            op_sequence,
            muscle_nominal_exposure_us,
            muscle_buffer_time_us)) {
        spdlog::error(*error);
        return error;
    }

    // Switch the free-running camera to the recording muscle frame rate before
    // START_RECORDING, so it is already emitting common-time onsets at the
    // recording cadence when the firmware begins locking the behavior frames to
    // them. This blocks until the camera server has applied the new exposure,
    // so START_RECORDING below is only sent afterwards.
    if (muscle_imaging_enabled_) {
        muscle_camera_->set_nominal_exposure_us(
            static_cast<unsigned int>(muscle_nominal_exposure_us));
    }

    // Initialize save directory
    save_directory_->initialize();

    // Number of muscle frames between periodic re-syncs of the muscle cameras,
    // computed once for the whole recording.
    const unsigned int musc_resync_interval = get_musc_resync_interval(
        musc_resync_interval_s_, params.behavior_fps, params.sync_ratio);

    // Save the recording metadata into the freshly created save directory.
    write_experiment_parameters(
        params,
        muscle_nominal_exposure_us,
        muscle_buffer_time_us,
        musc_resync_interval);
    write_recorder_config();
    write_behavior_calibration_parameters();
    write_muscle_camera_rois();
    copy_homography_parameters_if_present();

    // Send triggering parameters and start recording. The controller reverts to
    // the streaming (revert-to) params when the recording ends.
    TriggerParams rec_params =
        build_recording_params(params, musc_resync_interval);
    TriggerParams revert_to_params = build_streaming_params();
    arduino_communication_->start_recording(
        rec_params, revert_to_params, op_sequence);

    // The controller waits cam_flush_time_us after START_RECORDING before it
    // starts triggering, so that frames acquired with the previous (streaming)
    // params drain out of the camera buffers. Ignore frames for a fraction of
    // that window on this side too, so the recording does not begin with stale
    // frames (see cam_flush_time_us in comm_protocol/protocol.h).
    std::this_thread::sleep_for(
        std::chrono::microseconds(cam_flush_time_us * 8 / 10));
    // Set the muscle-imaging flag before raising is_recording so the muscle
    // acquirer sees a consistent state: when muscle imaging is off the camera
    // still free-runs but its frames are not saved (behavior-only recording).
    program_state_->muscle_imaging_enabled.store(muscle_imaging_enabled_);
    stage_timing_start();
    program_state_->is_recording.store(true);

    spdlog::info(
        "Recording STARTED: {} recording, muscle imaging {}. Saving to '{}'",
        current_recording_is_scheduled_ ? "scheduled" : "open",
        muscle_imaging_enabled_ ? "enabled" : "disabled",
        save_directory_->get_directory().string());
    return std::nullopt;
}

void Recorder::write_experiment_parameters(
    const RecordingParams &params,
    int muscle_nominal_exposure_us,
    int muscle_buffer_time_us,
    unsigned int musc_resync_interval) {
    std::filesystem::path output_path = save_directory_->get_directory() /
                                        "metadata/experiment_parameters.yaml";

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "behavior_fps" << YAML::Value << params.behavior_fps;
    out << YAML::Key << "muscle_imaging_enabled" << YAML::Value
        << muscle_imaging_enabled_;
    out << YAML::Key << "muscle_sync_ratio" << YAML::Value << params.sync_ratio;
    out << YAML::Key << "behavior_exposure_time_us" << YAML::Value
        << params.behavior_exposure_time_us;
    out << YAML::Key << "muscle_light_on_time_us" << YAML::Value
        << params.muscle_light_on_time_us;
    // The derived continuous-mode (auto-sequence) timing -- the nominal
    // per-line exposure programmed into the camera and the slack in the
    // common-time window beyond the light-on time -- is only meaningful when
    // imaging muscle.
    if (muscle_imaging_enabled_) {
        out << YAML::Key << "muscle_nominal_exposure_us" << YAML::Value
            << muscle_nominal_exposure_us;
        out << YAML::Key << "muscle_buffer_time_us" << YAML::Value
            << muscle_buffer_time_us;
        // The muscle cameras are re-synced every this many muscle frames,
        // which costs one muscle frame interval each (no frames are lost).
        out << YAML::Key << "muscle_resync_interval_frames" << YAML::Value
            << musc_resync_interval;
    }
    out << YAML::Key << "experiment_protocol" << YAML::Value
        << params.experiment_protocol;
    out << YAML::EndMap;

    std::ofstream fout(output_path);
    if (!fout.is_open()) {
        std::string error_message =
            "Failed to open file for writing experiment parameters: " +
            output_path.string();
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }
    fout << out.c_str();
    fout.close();

    spdlog::info("Saved experiment parameters to '{}'", output_path.string());
}

void Recorder::write_recorder_config() {
    std::filesystem::path output_path =
        save_directory_->get_directory() / "metadata/recorder_config.yaml";
    recorder_config_.save_to_file(output_path);
    spdlog::info("Saved recorder config to '{}'", output_path.string());
}

void Recorder::write_behavior_calibration_parameters() {
    std::filesystem::path output_path =
        save_directory_->get_directory() /
        "metadata/calibration_parameters_behavior.yaml";
    behavior_cam_calibration_params_.save_to_file(output_path);
    spdlog::info(
        "Saved behavior calibration parameters to '{}'", output_path.string());
}

void Recorder::write_muscle_camera_rois() {
    if (!muscle_camera_) {
        return;
    }
    std::filesystem::path output_path =
        save_directory_->get_directory() / "metadata/muscle_camera_roi.yaml";
    if (muscle_camera_->get_rois().to_file(output_path) == 0) {
        spdlog::info("Saved muscle camera ROIs to '{}'", output_path.string());
    }
}

void Recorder::copy_homography_parameters_if_present() {
    std::filesystem::path src =
        profile_dir_ /
        "calibration/model/homography_consensus/homography_result.yaml";
    if (!std::filesystem::exists(src)) {
        spdlog::warn(
            "No homography calibration found at '{}'. Muscle-to-behavior "
            "alignment will not be available for this recording unless "
            "homography_parameters.yaml is provided manually.",
            src.string());
        return;
    }

    std::filesystem::path dst =
        save_directory_->get_directory() / "metadata/homography_parameters.yaml";
    if (std::filesystem::exists(dst)) {
        spdlog::warn(
            "Overwriting existing '{}'. If that file was from a different "
            "calibration run, muscle alignment for this recording may use "
            "incorrect parameters.",
            dst.string());
    }
    std::filesystem::copy_file(
        src, dst, std::filesystem::copy_options::overwrite_existing);
    spdlog::info("Snapshotted homography parameters to '{}'", dst.string());
}

void Recorder::end_recording(bool reached_programmed_end) {
    if (reached_programmed_end) {
        // A scheduled recording reached its end: the controller already
        // reverted to the streaming params via its opSequence STOP step, so
        // there is nothing to send (a STOP_RECORDING here would fault the
        // controller).
    } else if (current_recording_is_scheduled_) {
        // Manual early abort of a scheduled recording. STOP_RECORDING is only
        // valid for an open recording, so revert by re-streaming instead.
        arduino_communication_->stream(build_streaming_params());
    } else {
        // Open recording: STOP_RECORDING reverts the controller to streaming
        // (using the revert-to params sent with START_RECORDING).
        arduino_communication_->stop_recording();
    }

    // Stop queuing frames. The acquirer threads flush any partial behavior
    // group and discard subsequent frames (see behavior_image_acquirer).
    program_state_->is_recording.store(false);

    // Revert the free-running muscle camera to the streaming muscle frame rate,
    // matching the streaming params the controller was just reverted to. This
    // blocks until the camera server has applied it.
    push_muscle_camera_exposure(streaming_behavior_fps_, streaming_sync_ratio_);

    // Re-stream now that the camera runs at the streaming rate. Until the new
    // exposure took effect, the camera still ran at the (possibly faster)
    // recording rate while the controller already expected streaming timing,
    // which can latch a muscle-frame overrun error. A STREAM clears that error
    // and restarts the controller's timing on the camera's new cadence.
    arduino_communication_->stream(build_streaming_params());

    spdlog::info(
        "Recording STOPPED ({}): {} recording, muscle imaging {}",
        reached_programmed_end ? "reached scheduled end"
                               : "user-initiated stop",
        current_recording_is_scheduled_ ? "scheduled" : "open",
        muscle_imaging_enabled_ ? "enabled" : "disabled");

    current_recording_is_scheduled_ = false;
}

bool Recorder::finalize_programmed_stop_if_reached() {
    // The acquirer threads stop recording exactly on the programmed frame count
    // by themselves; this only reverts the cameras to streaming shortly after.
    if (!programmed_recording_stop_->programmed_stop_reached.load()) {
        return false;
    }
    spdlog::info("Protocol stop reached. Finalizing recording.");
    end_recording(true);
    programmed_recording_stop_->num_behavior_frames_expected = -1;
    programmed_recording_stop_->num_muscle_frames_expected = -1;
    programmed_recording_stop_->programmed_stop_reached.store(false);
    return true;
}

void Recorder::wait_until_queues_drained() const {
    auto queue_is_empty = [](std::mutex &mutex, const auto &queue) {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    };
    while (!queue_is_empty(
               behavior_recording_state_->behavior_image_queue_mutex,
               behavior_recording_state_->behavior_image_queue) ||
           !queue_is_empty(
               muscle_recording_state_->muscle_image_queue_mutex,
               muscle_recording_state_->muscle_image_queue)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Quit by explicitly stopping acquisition on the behavior camera* and
// terminating the PCO server, then telling the saver and control threads that
// the work is done, and finally std::exit()-ing.
//
// * Without stopping acquisition explicitly, the frame grabber will think the
// device is still busy the next time we run the program.
//
// Teardown ends in std::exit() rather than joining the worker threads: some of
// them are intentionally parked in blocking waits (e.g. the muscle acquirer in
// pthread_cond_wait once its server is gone), and the critical hardware -- the
// Euresys grabber and the PCO server -- is already released above, so the
// remaining threads are safely reaped by process exit.
void Recorder::quit(int exit_code) {
    stage_timing_log();
    spdlog::info("Initiating shutdown");

    program_state_->to_quit.store(true);

    // Stop behavior camera acquisition
    if (std::shared_ptr<BehaviorCamera> behavior_camera =
            behavior_recording_state_->behavior_camera.load()) {
        spdlog::info("Stopping acquisition on behavior camera");
        behavior_camera->stop();
    }

    // Terminate both PCO camera servers. stop() is bounded (SIGTERM, then
    // SIGKILL after a grace period), so an unresponsive server can never block
    // the shutdown indefinitely. Once the servers are gone the muscle acquirer
    // blocks forever in wait_for_next_frame_pair()'s pthread_cond_wait; that
    // thread is abandoned at std::exit() below.
    if (muscle_camera_) {
        spdlog::info("Stopping acquisition on muscle camera");
        muscle_camera_->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stop_motion_control_request_handler(program_state_);

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stop_behavior_image_saver(behavior_recording_state_, program_state_);

    spdlog::info("Telling muscle image saver threads to stop.");
    stop_muscle_image_saver(muscle_recording_state_, program_state_);

    // Stop muscle excitation
    spdlog::info("Switching off muscle excitation light.");
    arduino_communication_->stop_excitation();

    // The worker threads -- joinable members -- are intentionally reaped by the
    // process exit rather than joined (see above). std::exit() does not unwind
    // the stack, so this object's destructor never runs.
    std::exit(exit_code);
}

void Recorder::push_muscle_camera_exposure(int beh_frame_rate, int sync_ratio) {
    if (!muscle_camera_) {
        return; // started with --no-muscle
    }
    // In continuous (auto-sequence) mode the camera free-runs at
    // 1/(nominalExposure + readout), so the nominal per-line exposure sets the
    // frame rate. Pick it so the camera produces muscle frames at
    // behFrameRate / syncRatio. See docs/data_acquisition.md and
    // MuscleTriggerTiming.
    unsigned int muscle_interval_us =
        static_cast<unsigned int>(1000000.0 * sync_ratio / beh_frame_rate);
    int exposure_us = static_cast<int>(muscle_interval_us) -
                      static_cast<int>(pco_cam_readout_time_us_);
    if (exposure_us <= 0) {
        spdlog::error(
            "Cannot set muscle camera exposure: muscle interval ({} us) is not "
            "longer than the sensor readout time ({} us).",
            muscle_interval_us,
            pco_cam_readout_time_us_);
        return;
    }
    muscle_camera_->set_nominal_exposure_us(
        static_cast<unsigned int>(exposure_us));
}

TriggerParams Recorder::build_streaming_params() const {
    TriggerParams params;
    // During streaming the controller always runs DEFAULT parameters. The
    // recording parameters (behavior FPS, sync ratio, behavior exposure, muscle
    // light-on) take effect only when a recording starts (see
    // build_recording_params). This keeps the controller's status display
    // showing the actual live behavior, never the not-yet-executed recording
    // config.
    //
    // The one live streaming control is muscle imaging: it toggles whether the
    // behavior camera is locked to the muscle camera and the blue excitation
    // LED is pulsed (enableMuscle), for muscle preview. When false, the
    // muscle-only fields are still sent but ignored by the controller.
    params.enable_muscle = muscle_imaging_enabled_;
    params.beh_frame_rate = streaming_behavior_fps_;
    params.beh_musc_sync_ratio = streaming_sync_ratio_;
    params.beh_exp_time = static_cast<unsigned int>(default_beh_exp_time_us_);
    params.musc_eff_exp_time =
        static_cast<unsigned int>(default_musc_light_on_time_us_);
    params.pco_cam_rolling_time = pco_cam_rolling_time_us_;
    params.pco_cam_readout_time = pco_cam_readout_time_us_;
    params.musc_resync_interval = get_musc_resync_interval(
        musc_resync_interval_s_, streaming_behavior_fps_, streaming_sync_ratio_);
    params.musc_acquire_restart_margin = musc_acquire_restart_margin_us_;
    return params;
}

TriggerParams Recorder::build_recording_params(
    const RecordingParams &params, unsigned int musc_resync_interval) const {
    TriggerParams trigger_params;
    // The muscle camera is recorded (and the blue excitation light pulsed) only
    // when muscle imaging is enabled; otherwise the controller free-runs the
    // behavior camera (enableMuscle = false).
    trigger_params.enable_muscle = muscle_imaging_enabled_;
    trigger_params.beh_frame_rate = params.behavior_fps;
    trigger_params.beh_musc_sync_ratio = params.sync_ratio;
    trigger_params.beh_exp_time =
        static_cast<unsigned int>(params.behavior_exposure_time_us);
    trigger_params.musc_eff_exp_time =
        static_cast<unsigned int>(params.muscle_light_on_time_us);
    trigger_params.pco_cam_rolling_time = pco_cam_rolling_time_us_;
    trigger_params.pco_cam_readout_time = pco_cam_readout_time_us_;
    trigger_params.musc_resync_interval = musc_resync_interval;
    trigger_params.musc_acquire_restart_margin =
        musc_acquire_restart_margin_us_;
    return trigger_params;
}

const char *const protocol_string_help =
    "It must be a ';'-separated list of steps, each of the form\n"
    "    frameIdx/channel/op\n"
    "where:\n"
    "  - frameIdx is a non-negative integer: the behavior-frame index "
    "after which the step is applied;\n"
    "  - to switch an optogenetics channel, channel is 'ch2' or 'ch3' "
    "(channel 1 is reserved for the IR LED) and op is 'on' or 'off';\n"
    "  - to end the recording, channel is 'x' and op is 'stop'.\n\n"
    "If any steps are given, the protocol must contain exactly one "
    "'x/stop' step, and it must be the very last step.\n\n"
    "Do not add a trailing ';' at the end.\n\n"
    "Leave empty for open recording (no programmed stop).\n\n"
    "Example:\n"
    "    300/ch2/on;600/ch2/off;900/x/stop\n"
    "turns channel 2 on after frame 300, off after frame 600, and "
    "stops the recording after frame 900.";

// The text is a ";"-separated list of steps, each "frameIdx/channel/op":
//   - "<n>/ch2/on", "<n>/ch3/off": toggle an optogenetics channel
//   - "<n>/x/stop": end the recording and revert to streaming
// An empty string (or a single ";") denotes an open recording with no
// programmed steps. If any steps are given, the protocol must contain exactly
// one "<n>/x/stop" step and it must be the very last step.
int parse_protocol_string(
    const std::string &protocol_text_field_string,
    std::deque<OperationStep> &op_sequence) {
    op_sequence.clear();

    auto report_error = []() {
        spdlog::error("Invalid experiment protocol string");
        return -1;
    };

    if (protocol_text_field_string == ";") {
        return 0;
    }

    std::istringstream stream(protocol_text_field_string);
    std::string token;
    int num_steps_parsed = 0;
    while (std::getline(stream, token, ';')) {
        if (token.empty()) {
            return report_error();
        }

        std::istringstream token_stream(token);
        std::string frame_str, channel_str, op_str;
        if (!std::getline(token_stream, frame_str, '/') ||
            !std::getline(token_stream, channel_str, '/') ||
            !std::getline(token_stream, op_str, '/')) {
            return report_error();
        }

        unsigned long frame_idx = 0;
        OptoChannel channel = OptoChannel::all;
        OpType op = OpType::stop;
        try {
            frame_idx = std::stoul(frame_str);
            if (channel_str == "x" && op_str == "stop") {
                channel = OptoChannel::all;
                op = OpType::stop;
            } else if (channel_str.rfind("ch", 0) == 0) {
                channel =
                    static_cast<OptoChannel>(std::stoi(channel_str.substr(2)));
                if (op_str == "on") {
                    op = OpType::on;
                } else if (op_str == "off") {
                    op = OpType::off;
                } else {
                    return report_error();
                }
            } else {
                return report_error();
            }
        } catch (const std::exception &) {
            return report_error();
        }

        OperationStep step(frame_idx, channel, op);
        if (!step.is_valid) {
            return report_error();
        }
        op_sequence.push_back(step);
        ++num_steps_parsed;
    }

    // If any steps are given, require exactly one stop step, at the very end.
    if (num_steps_parsed > 0) {
        int num_stops = 0;
        for (const OperationStep &step : op_sequence) {
            if (step.op == OpType::stop) {
                ++num_stops;
            }
        }
        if (num_stops != 1 || op_sequence.back().op != OpType::stop) {
            return report_error();
        }
    }

    return num_steps_parsed;
}
