/**
 * run-spotlight -- main recording application.
 *
 * Loads the recorder config from <profile_dir>/recorder_config.yaml, the
 * muscle-camera ROI from <profile_dir>/muscle_camera_roi.yaml, and the
 * arena registration model from <arena_dir>/model/calibration_result.yaml.
 * Reads arena dimensions from <arena_dir>/metadata.yaml to derive the stage
 * range used by the motion-stage preview widget.
 *
 * Launches all acquisition, saving, tracking, and Arduino threads and opens the
 * main GUI window. Whether muscle is imaged, and the muscle/behavior frame
 * rates and exposures, are controlled from the main GUI window at runtime.
 *
 * CLI:  run-spotlight -p PROFILE_DIR -a ARENA_DIR [OPTIONS]
 *       (see --help for details; -a/--arena is required)
 */
#include "recorder/apps/run_spotlight.h"

namespace {
QApplication *application = nullptr;
MainGUIWindow *main_gui_window = nullptr;
std::shared_ptr<ProgramState> program_state;
std::shared_ptr<BehaviorRecordingState> behavior_recording_state;
std::shared_ptr<MuscleRecordingState> muscle_recording_state;
std::shared_ptr<ArduinoCommunication> arduino_communication;

// Stage range (mm) covering the arena, for the motion-stage preview widget.
struct StageRange {
    double min_x_mm, max_x_mm, min_y_mm, max_y_mm;
};

StageRange compute_stage_range_from_arena(
    const std::filesystem::path &arena_dir,
    const RecorderConfig &recorder_config,
    const CalibrationParams &behavior_cam_calibration_params)
/**
 * Compute the stage range covering the arena, for the motion-stage preview
 * widget. Read arena dimensions from <arenaDir>/metadata.yaml, then invert the
 * calibration model at the image center to find which stage position
 * corresponds to each of the four arena corners. The computed limits are
 * clipped to [0, physical_range_limit_mm].
 *
 * Side effect: registers the clipped limits as software motion stage limits via
 * set_motion_stage_limits() so set_target_motion_stage_position() clamps.
 */
{
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
    stage_min_x_mm = clip_to_physical_range(stage_min_x_mm, "stageMinX");
    stage_max_x_mm = clip_to_physical_range(stage_max_x_mm, "stageMaxX");
    stage_min_y_mm = clip_to_physical_range(stage_min_y_mm, "stageMinY");
    stage_max_y_mm = clip_to_physical_range(stage_max_y_mm, "stageMaxY");
    set_motion_stage_limits(
        stage_min_x_mm, stage_max_x_mm, stage_min_y_mm, stage_max_y_mm);

    return {stage_min_x_mm, stage_max_x_mm, stage_min_y_mm, stage_max_y_mm};
}

void join_if_joinable(std::thread &thread, const char *name)
/**
 * Join `thread` if it is joinable, logging before and after.
 */
{
    spdlog::debug("Waiting for {} to finish", name);
    if (thread.joinable()) {
        thread.join();
    }
    spdlog::debug("{} finished", name);
}
} // namespace

bool quit_program()
/**
 * Quit gracefully by explicitly stopping acquisition on the behavior
 * camera* and telling saver threads that the work is done.
 *
 * * Without stopping acquisition explicitly, the frame grabber will
 * think the device is still busy the next time we run the program.
 */
{
    spdlog::info("SIGINT received. Initiating graceful shutdown");

    program_state->to_quit.store(true);

    // Stop behavior camera acquisition
    if (behavior_recording_state->behavior_camera) {
        spdlog::info("Stopping acquisition on behavior camera");
        behavior_recording_state->behavior_camera->stop();
    }

    // Terminate PCO camera server. stop() is bounded (SIGTERM, then SIGKILL
    // after a grace period), so an unresponsive server can never block the
    // shutdown indefinitely.
    //
    // We deliberately do NOT reset muscleRecordingState->muscleCamera here: the
    // muscle acquirer thread dereferences that shared_ptr without taking its
    // own copy, so destroying the MuscleCamera now would be a use-after-free.
    // Once the server is gone the acquirer blocks forever in
    // wait_for_one_frame()'s pthread_cond_wait, but it is abandoned at
    // std::exit() below, with the MuscleCamera object left alive and valid
    // until the process exits.
    if (muscle_recording_state->muscle_camera) {
        spdlog::info("Stopping acquisition on muscle camera");
        muscle_recording_state->muscle_camera->stop();
    }

    // Tell motion control request handler thread to stop
    spdlog::info("Telling motion control request handler thread to stop.");
    stop_motion_control_request_handler(program_state);

    // Tell behavior camera saver threads to stop
    spdlog::info("Telling behavior image saver threads to stop.");
    stop_behavior_image_saver(behavior_recording_state, program_state);

    spdlog::info("Telling muscle image saver threads to stop.");
    stop_muscle_image_saver(muscle_recording_state, program_state);

    // Stop muscle excitation
    spdlog::info("Switching off muscle excitation light.");
    arduino_communication->stop_excitation();

    std::exit(0);
}

int run_spotlight_main(int argc, char **argv) {
    std::signal(SIGINT, [](int) { quit_program(); });

    // Parse command line arguments
    CLIOptions options = parse_cli(argc, argv);
    if (options.arena_dir.empty()) {
        std::cerr << "Error: -a/--arena is required (path to the arena "
                     "directory containing metadata.yaml and model/).\n";
        return 1;
    }

    // Set log level based on CLI options
    spdlog::set_level(options.log_level);

    QApplication local_application(argc, argv);
    application = &local_application;

    // Make program state holder
    program_state = std::make_shared<ProgramState>();

    // Load recorder configuration
    std::filesystem::path profile_dir =
        std::filesystem::path(expand_path(options.profile_dir));
    std::filesystem::path arena_dir =
        std::filesystem::path(expand_path(options.arena_dir));
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "runSpotlight main loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);
    if (!recorder_config.is_defined) {
        std::string error_message = fmt::format(
            "Failed to load recorder configuration. Cannot start recording. "
            "Expected valid recorder configuration file at {}",
            config_path.c_str());
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }

    // Load muscle ROI
    std::filesystem::path roi_file_path =
        profile_dir / "muscle_camera_roi.yaml";
    MuscleCameraROI muscle_roi = get_muscle_camera_roi(roi_file_path);

    // Make atomic variable that holds the save directory
    std::string default_save_directory =
        recorder_config.get_parameter<std::string>("gui", "default_save_dir");
    std::shared_ptr<SaveDirectory> save_directory =
        std::make_shared<SaveDirectory>(default_save_directory);

    // Load position mapping / calibration parameters from the arena
    // registration scan (`fit-arena-registration` output).
    std::filesystem::path calibration_params_file_path =
        arena_dir / "model" / "calibration_result.yaml";
    spdlog::info(
        "Loading spatial calibration parameters from {}",
        calibration_params_file_path.string());
    CalibrationParams behavior_cam_calibration_params(
        calibration_params_file_path.string());
    if (!behavior_cam_calibration_params.is_defined) {
        std::string error_message = fmt::format(
            "Spatial calibration data not found or malformed. This is required "
            "for tracking and recording. Expected valid calibration file at {} "
            "(produced by `fit-arena-registration -a <arena_dir>`).",
            calibration_params_file_path.string());
        spdlog::critical(error_message);
        throw std::runtime_error(error_message);
    }

    // Load the active-area mask for closed-loop tracking.
    double boundary_margin_mm =
        recorder_config.get_parameter<double>("tracking", "boundary_margin_mm");
    ActiveAreaMask active_area_mask(
        arena_dir.string(),
        boundary_margin_mm,
        behavior_cam_calibration_params.stage_and_pixel_to_physical);
    spdlog::info("Loaded active area mask from {}", arena_dir.string());

    // Compute the stage range covering the arena, for the motion-stage preview
    // widget. Also registers software motion stage limits as a side effect.
    StageRange stage_range = compute_stage_range_from_arena(
        arena_dir, recorder_config, behavior_cam_calibration_params);

    // Initialize behavior and muscle imaging states
    behavior_recording_state = std::make_shared<BehaviorRecordingState>();
    behavior_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();
    muscle_recording_state = std::make_shared<MuscleRecordingState>();
    muscle_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();

    // Start tracking & motion control threads
    std::shared_ptr<TrackingControlState> tracking_control_state =
        std::make_shared<TrackingControlState>();
    std::thread motion_control_io_thread(
        motion_control_request_handler,
        recorder_config,
        tracking_control_state,
        program_state);
    std::thread motion_stage_position_logger_thread(
        motion_stage_position_logger,
        recorder_config,
        tracking_control_state,
        save_directory,
        program_state);
    std::thread tracking_controller_thread(
        tracking_controller,
        recorder_config,
        std::ref(active_area_mask),
        behavior_recording_state,
        tracking_control_state,
        std::ref(behavior_cam_calibration_params),
        program_state);

    // Start behavior image acquirer
    std::shared_ptr<ProgrammedStop> programmed_recording_stop =
        std::make_shared<ProgrammedStop>();

    std::thread behavior_image_acquirer_thread(
        behavior_image_acquirer,
        recorder_config,
        behavior_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Behavior camera acquisition thread started");

    // Start behavior image savers
    std::vector<std::thread> behavior_image_saver_threads;
    int num_behavior_image_saver_threads = recorder_config.get_parameter<int>(
        "behavior_camera", "num_image_saving_threads");
    for (int i = 0; i < num_behavior_image_saver_threads; i++) {
        behavior_image_saver_threads.push_back(std::thread(
            behavior_image_saver,
            recorder_config,
            behavior_recording_state,
            save_directory,
            program_state));
    }
    spdlog::info("Behavior camera saver threads started");

    // Start muscle image acquirer
    spdlog::info(
        "Loaded muscle camera ROI from {}: x0={}, x1={}, y0={}, y1={} "
        "(x_offset={}, y_offset={}, image_width={}, image_height={})",
        roi_file_path.string(),
        muscle_roi.x0,
        muscle_roi.x1,
        muscle_roi.y0,
        muscle_roi.y1,
        muscle_roi.x_offset,
        muscle_roi.y_offset,
        muscle_roi.image_width,
        muscle_roi.image_height);
    std::thread muscle_image_acquirer_thread(
        muscle_image_acquirer,
        muscle_roi.image_width,
        muscle_roi.image_height,
        muscle_roi.x_offset,
        muscle_roi.y_offset,
        recorder_config,
        profile_dir,
        spdlog::get_level(),
        muscle_recording_state,
        program_state,
        programmed_recording_stop);
    spdlog::info("Muscle camera acquisition thread started");
    size_t retry_count = 0;
    while (!muscle_recording_state->muscle_camera) {
        // Wait for the muscle camera to be initialized
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        spdlog::warn("Waiting for muscle camera to be initialized...");
        retry_count++;
        if (retry_count % 10 == 0) {
            spdlog::warn("Muscle camera is not initialized.");
        }
    }
    // The muscle camera's shutter-open window is configured from the main GUI
    // window (initialized to, and tracking, the muscle light-on time spin box).

    // Start muscle image savers
    std::vector<std::thread> muscle_image_saver_threads;
    int num_muscle_image_saver_threads = recorder_config.get_parameter<int>(
        "muscle_camera", "num_image_saving_threads");
    for (int i = 0; i < num_muscle_image_saver_threads; i++) {
        muscle_image_saver_threads.push_back(std::thread(
            muscle_image_saver,
            recorder_config,
            muscle_recording_state,
            save_directory,
            program_state,
            5)); // cv::IMWRITE_TIFF_COMPRESSION_LZW
    }
    spdlog::info("Muscle camera saver threads started");

    // Start Arduino triggering interface
    std::string arduino_port_name = find_arduino_port_name(recorder_config);
    arduino_communication =
        std::make_shared<ArduinoCommunication>(arduino_port_name);

    // Reboot the trigger controller into a clean, known state at startup. The
    // comm thread waits for the reboot and reopens the port, so the GUI's first
    // STREAM (sent when the main window is constructed below) reaches the
    // freshly reset controller.
    arduino_communication->reset();

    // Create and show GUI
    MainGUIWindow local_main_gui_window(
        recorder_config,
        behavior_recording_state,
        muscle_recording_state,
        tracking_control_state,
        std::ref(behavior_cam_calibration_params),
        save_directory,
        arduino_communication,
        program_state,
        programmed_recording_stop,
        active_area_mask,
        stage_range.min_x_mm,
        stage_range.max_x_mm,
        stage_range.min_y_mm,
        stage_range.max_y_mm,
        nullptr);
    main_gui_window = &local_main_gui_window;
    main_gui_window->show();

    int result = application->exec();

    // Wait for threads to finish
    join_if_joinable(
        behavior_image_acquirer_thread, "behavior image acquirer thread");

    for (auto &thread : behavior_image_saver_threads) {
        join_if_joinable(thread, "one of the behavior image saver threads");
    }

    join_if_joinable(
        muscle_image_acquirer_thread, "muscle image acquirer thread");

    for (auto &thread : muscle_image_saver_threads) {
        join_if_joinable(thread, "one of the muscle image saver threads");
    }

    join_if_joinable(motion_control_io_thread, "motion control IO thread");
    join_if_joinable(
        motion_stage_position_logger_thread,
        "motion stage position logger thread");
    join_if_joinable(tracking_controller_thread, "tracking controller thread");

    return result;
}

int main(int argc, char **argv) {
    try {
        return run_spotlight_main(argc, argv);
    } catch (const std::exception &e) {
        spdlog::critical("Fatal startup error: {}", e.what());
        return 1;
    }
}