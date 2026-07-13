/**
 * run-charuco-homography-scan
 *
 * Performs a grid scan over a small stage region to collect paired
 * behavior+muscle images for ChArUco-based homography fitting.
 *
 * Workflow
 * --------
 * 1. Both cameras are started and Arduino triggering is enabled (muscle imaging
 *    on, sync_ratio=1).
 * 2. The stage visits each position of a serpentine grid defined by
 *    [homography] scan_x_center_mm, scan_y_center_mm, scan_range_mm,
 *    scan_stride_mm in recorder_config.yaml.
 * 3. At each position the stage is allowed to settle (200 ms sleep +
 *    scan_settling_frames settling frames dropped from each camera), then
 *    10 frame pairs are captured.
 * 4. Images are saved to:
 *      <profile_dir>/calibration/charuco_homography_scan/behavior_camera/
 *          homography_scan_x{x:.2f}_y{y:.2f}_frame{i}.jpg
 *      <profile_dir>/calibration/charuco_homography_scan/muscle_camera/
 *          homography_scan_x{x:.2f}_y{y:.2f}_frame{i}.tif
 *
 * After the scan, run `fit_homography.py --profile-dir <profile_dir>` to fit
 * the homography model.
 *
 * CLI:  run-charuco-homography-scan -p PROFILE_DIR [OPTIONS]
 *       (see --help for details)
 */
#include "recorder/apps/run_charuco_homography_scan.h"

#include <csignal>
#include <filesystem>
#include <iostream>
#include <queue>
#include <thread>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

namespace {

std::shared_ptr<ProgramState> program_state = nullptr;
std::unique_ptr<ArduinoCommunication> arduino_communication = nullptr;
std::shared_ptr<BehaviorRecordingState> behavior_recording_state = nullptr;
std::shared_ptr<MuscleRecordingState> muscle_recording_state = nullptr;

volatile std::sig_atomic_t interrupt_requested = 0;

// Block until a frame with a received_time different from after_time arrives.
FrameData wait_for_next_frame(
    const std::shared_ptr<LatestFrame> &latest_frame_holder,
    uint64_t after_time) {
    FrameData frame;
    do {
        frame = latest_frame_holder->get_latest_frame_data();
        if (frame.received_time != after_time)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (true);
    return frame;
}

// Build a serpentine (boustrophedon) grid of stage positions.
// Scans from (x_center-range, y_center-range) to (x_center+range,
// y_center+range), alternating column direction so stage travel is minimised.
std::queue<std::pair<double, double>> make_grid(
    double x_center, double y_center, double range, double stride) {
    std::queue<std::pair<double, double>> positions;

    double x_min = x_center - range;
    double x_max = x_center + range;
    double y_min = y_center - range;
    double y_max = y_center + range;

    int n_cols = static_cast<int>(std::round((x_max - x_min) / stride)) + 1;
    int n_rows = static_cast<int>(std::round((y_max - y_min) / stride)) + 1;

    for (int col = 0; col < n_cols; ++col) {
        double x = x_min + col * stride;
        for (int row = 0; row < n_rows; ++row) {
            double y = (col % 2 == 0) ? y_min + row * stride
                                      : y_max - row * stride;
            positions.push({x, y});
        }
    }
    return positions;
}

// Clear and (re)create a directory, removing all files inside.
void reset_output_dir(const std::filesystem::path &dir) {
    if (std::filesystem::exists(dir)) {
        for (auto &entry : std::filesystem::directory_iterator(dir)) {
            if (entry.is_regular_file())
                std::filesystem::remove(entry.path());
        }
    } else {
        std::filesystem::create_directories(dir);
    }
}

} // namespace

void run_charuco_homography_scan(const std::filesystem::path &profile_dir) {
    // ------------------------------------------------------------------
    // Load config
    // ------------------------------------------------------------------
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "run-charuco-homography-scan loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);

    // Read homography scan parameters from the dedicated [homography] section.
    double x_center = recorder_config.get_parameter<double>(
        "homography", "scan_x_center_mm");
    double y_center = recorder_config.get_parameter<double>(
        "homography", "scan_y_center_mm");
    double range =
        recorder_config.get_parameter<double>("homography", "scan_range_mm");
    double stride =
        recorder_config.get_parameter<double>("homography", "scan_stride_mm");
    int settling_frames = recorder_config.get_parameter<int>(
        "homography", "scan_settling_frames");

    double motion_velocity = recorder_config.get_parameter<double>(
        "motion_control", "default_velocity_mm_per_s");

    // ------------------------------------------------------------------
    // Prepare output directories (clear old images to avoid stale data)
    // ------------------------------------------------------------------
    std::filesystem::path scan_root =
        profile_dir / "calibration/charuco_homography_scan";
    std::filesystem::path beh_out = scan_root / "behavior_camera";
    std::filesystem::path mus_out = scan_root / "muscle_camera";
    reset_output_dir(beh_out);
    reset_output_dir(mus_out);
    spdlog::info(
        "Output directories prepared: {}", scan_root.string());

    // ------------------------------------------------------------------
    // Build the scan grid
    // ------------------------------------------------------------------
    std::queue<std::pair<double, double>> grid = make_grid(
        x_center, y_center, range, stride);
    spdlog::info(
        "Grid: center=({:.1f}, {:.1f}) mm, range=±{:.1f} mm, "
        "stride={:.1f} mm → {} positions",
        x_center, y_center, range, stride, grid.size());

    // ------------------------------------------------------------------
    // Load muscle camera ROI
    // ------------------------------------------------------------------
    std::filesystem::path roi_path = profile_dir / "muscle_camera_roi.yaml";
    MuscleCameraROI muscle_roi = get_muscle_camera_roi(roi_path);
    spdlog::info(
        "Muscle camera ROI: x0={}, x1={}, y0={}, y1={} "
        "(width={}, height={}, xOffset={}, yOffset={})",
        muscle_roi.x0, muscle_roi.x1, muscle_roi.y0, muscle_roi.y1,
        muscle_roi.image_width, muscle_roi.image_height,
        muscle_roi.x_offset, muscle_roi.y_offset);

    // ------------------------------------------------------------------
    // Start camera acquisition threads
    // ------------------------------------------------------------------
    program_state = std::make_shared<ProgramState>();
    auto programmed_stop = std::make_shared<ProgrammedStop>();

    behavior_recording_state = std::make_shared<BehaviorRecordingState>();
    behavior_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();

    spdlog::info("Starting behavior camera acquisition thread");
    std::thread behavior_thread(
        behavior_image_acquirer,
        recorder_config,
        behavior_recording_state,
        program_state,
        programmed_stop);

    muscle_recording_state = std::make_shared<MuscleRecordingState>();
    muscle_recording_state->latest_frame_holder =
        std::make_shared<LatestFrame>();

    spdlog::info("Starting muscle camera acquisition thread");
    std::thread muscle_thread(
        muscle_image_acquirer,
        static_cast<unsigned int>(muscle_roi.image_width),
        static_cast<unsigned int>(muscle_roi.image_height),
        static_cast<unsigned int>(muscle_roi.x_offset),
        static_cast<unsigned int>(muscle_roi.y_offset),
        recorder_config,
        profile_dir.string(),
        spdlog::get_level(),
        muscle_recording_state,
        program_state,
        programmed_stop);

    // ------------------------------------------------------------------
    // Wait for both cameras to be ready
    // ------------------------------------------------------------------
    {
        size_t retry = 0;
        while (true) {
            auto cam = behavior_recording_state->behavior_camera.load();
            if (cam && cam->is_ready())
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (++retry % 20 == 0)
                spdlog::warn("Waiting for behavior camera to initialize...");
        }
        spdlog::info("Behavior camera ready");
    }
    {
        size_t retry = 0;
        while (!muscle_recording_state->muscle_camera.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (++retry % 20 == 0)
                spdlog::warn("Waiting for muscle camera to initialize...");
        }
        spdlog::info("Muscle camera ready");
    }

    // ------------------------------------------------------------------
    // Start Arduino triggering with muscle imaging on
    // ------------------------------------------------------------------
    int muscle_num_lines_scanned =
        muscle_recording_state->muscle_camera.load()->get_num_lines_scanned();
    arduino_communication = initialize_triggering_with_default_params(
        recorder_config,
        muscle_num_lines_scanned,
        /*sync_ratio=*/1,
        /*muscle_imaging_on=*/true);

    // ------------------------------------------------------------------
    // Set up motion control and shutdown lambda
    // ------------------------------------------------------------------
    MotionControl motion_control(recorder_config);

    auto shutdown = [&]() {
        program_state->to_quit.store(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (auto cam = behavior_recording_state->behavior_camera.load()) {
            cam->stop();
            behavior_recording_state->behavior_camera.store(nullptr);
        }
        muscle_recording_state->muscle_camera.store(nullptr);
        if (behavior_thread.joinable())
            behavior_thread.join();
        if (muscle_thread.joinable())
            muscle_thread.join();
        arduino_communication->stop_excitation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduino_communication->stop_communication();
    };

    // ------------------------------------------------------------------
    // Scan loop
    // ------------------------------------------------------------------
    const std::vector<int> jpeg_params = {
        cv::IMWRITE_JPEG_QUALITY, 95};
    const int frames_per_position = 10;
    const int settling_sleep_ms = 200;

    int position_count = 0;
    int total_positions = static_cast<int>(grid.size());

    while (!grid.empty()) {
        if (interrupt_requested) {
            spdlog::info("Interrupt requested, stopping scan");
            break;
        }

        auto [target_x, target_y] = grid.front();
        grid.pop();
        ++position_count;

        spdlog::info(
            "Position {}/{}: moving to ({:.2f}, {:.2f}) mm",
            position_count, total_positions, target_x, target_y);

        // Move stage (non-blocking)
        try {
            motion_control.move_absolute(
                x_axis, target_x, false, motion_velocity);
            motion_control.move_absolute(
                y_axis, target_y, false, motion_velocity);
        } catch (const std::exception &e) {
            spdlog::error(
                "Stage move failed for ({:.2f}, {:.2f}): {}",
                target_x, target_y, e.what());
            shutdown();
            return;
        }

        // Wait until both axes are idle
        while (!motion_control.check_if_idle(x_axis) ||
               !motion_control.check_if_idle(y_axis)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Mechanical vibration settling sleep
        std::this_thread::sleep_for(
            std::chrono::milliseconds(settling_sleep_ms));

        // Drop settling frames from both cameras
        for (int i = 0; i < settling_frames; ++i) {
            uint64_t beh_time = behavior_recording_state->latest_frame_holder
                                    ->get_latest_frame_data()
                                    .received_time;
            uint64_t mus_time = muscle_recording_state->latest_frame_holder
                                    ->get_latest_frame_data()
                                    .received_time;
            wait_for_next_frame(
                behavior_recording_state->latest_frame_holder, beh_time);
            wait_for_next_frame(
                muscle_recording_state->latest_frame_holder, mus_time);
        }
        spdlog::debug(
            "Position ({:.2f}, {:.2f}): dropped {} settling frames",
            target_x, target_y, settling_frames);

        // Capture 10 frame pairs
        for (int frame_idx = 0; frame_idx < frames_per_position; ++frame_idx) {
            uint64_t beh_time = behavior_recording_state->latest_frame_holder
                                    ->get_latest_frame_data()
                                    .received_time;
            uint64_t mus_time = muscle_recording_state->latest_frame_holder
                                    ->get_latest_frame_data()
                                    .received_time;

            FrameData beh_frame = wait_for_next_frame(
                behavior_recording_state->latest_frame_holder, beh_time);
            FrameData mus_frame = wait_for_next_frame(
                muscle_recording_state->latest_frame_holder, mus_time);

            cv::Mat beh_img, mus_img;
            reorient_behavior_image(beh_frame.image, beh_img);
            reorient_muscle_image(mus_frame.image, mus_img);

            std::string stem = fmt::format(
                "homography_scan_x{:.2f}_y{:.2f}_frame{}",
                target_x, target_y, frame_idx);

            cv::imwrite(
                (beh_out / (stem + ".jpg")).string(), beh_img, jpeg_params);
            cv::imwrite(
                (mus_out / (stem + ".tif")).string(), mus_img);
        }

        spdlog::info(
            "Position ({:.2f}, {:.2f}): saved {} frame pairs",
            target_x, target_y, frames_per_position);
    }

    spdlog::info("Scan complete ({} positions)", position_count);

    // ------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------
    shutdown();
}

int main(int argc, char **argv) {
    std::signal(SIGINT, [](int) {
        interrupt_requested = 1;
        std::signal(SIGINT, SIG_DFL);
    });

    std::string profile_dir_str = "~/Spotlight/default/";
    spdlog::level::level_enum log_level = spdlog::level::info;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            // clang-format off
            std::cout
                << "Usage: " << argv[0]
                << " [OPTIONS]\n"
                << "  -p, --profile-dir PATH  Profile directory "
                   "(default: ~/Spotlight/default/)\n"
                << "  -v, --verbose           Debug-level logging\n"
                << "  --verbosity LEVEL       "
                   "trace/debug/info/warn/error/critical/off\n";
            // clang-format on
            return 0;
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc) {
            profile_dir_str = argv[++i];
        } else if (arg == "-v" || arg == "--verbose") {
            log_level = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            std::string lvl = argv[++i];
            if (lvl == "trace")
                log_level = spdlog::level::trace;
            else if (lvl == "debug")
                log_level = spdlog::level::debug;
            else if (lvl == "warn")
                log_level = spdlog::level::warn;
            else if (lvl == "error")
                log_level = spdlog::level::err;
            else if (lvl == "critical")
                log_level = spdlog::level::critical;
            else if (lvl == "off")
                log_level = spdlog::level::off;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    spdlog::set_level(log_level);
    std::filesystem::path profile_dir(expand_path(profile_dir_str));
    run_charuco_homography_scan(profile_dir);
    spdlog::info("Homography scan complete");

    return 0;
}
