/**
 * run-arena-registration-scan
 *
 * Performs the arena registration scan needed to fit the
 * (stage, pixel) <-> physical mapping model.
 *
 * Workflow
 * --------
 * 1. Live preview with crosshairs -- the user centres the camera on the
 *    DataMatrix barcode printed on the mapping board, then presses ENTER.
 * 2. The DataMatrix is decoded and its checksum is verified against
 *    <arena_dir>/metadata.yaml to confirm the correct arena is mounted.
 * 3. The stage-to-arena offset is computed from the current stage position
 *    and the known DataMatrix position in arena coordinates.
 * 4. The stage visits each AprilTag position in sequence, acquires 10
 *    consecutive frames per tag (after dropping a configurable number of
 *    settling frames), and saves:
 *      <arena_dir>/mapping_scan/apriltag<id>_img<i>.jpg
 *      <arena_dir>/mapping_scan/apriltag_stage_positions.csv
 *
 * After the scan, run `fit-arena-registration -a <arena_dir>` to fit the
 * calibration model.
 *
 * CLI:  run-arena-registration-scan -p PROFILE_DIR -a ARENA_DIR [OPTIONS]
 *       (see --help for details)
 *
 * Requires libdmtx-dev: sudo apt install libdmtx-dev
 */
// Requires libdmtx-dev: sudo apt install libdmtx-dev
#include "recorder/apps/run_arena_registration_scan.h"

#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>

#include <dmtx.h>
#include <yaml-cpp/yaml.h>
#include <zaber/motion/exceptions/bad_data_exception.h>

namespace {
std::shared_ptr<ProgramState> program_state = nullptr;
std::unique_ptr<ArduinoCommunication> arduino_communication = nullptr;
std::shared_ptr<BehaviorRecordingState> behavior_recording_state = nullptr;

void quit_program() {
    spdlog::info("SIGINT received. Initiating graceful shutdown");
    if (program_state)
        program_state->to_quit.store(true);
    if (arduino_communication) {
        // The new protocol has no "stop triggering" command; switch the blue
        // excitation light off, then close the link.
        arduino_communication->stop_excitation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduino_communication->stop_communication();
    }
    std::exit(0);
}

// Single decode attempt at a given shrink factor.
std::string
try_decode_data_matrix(const cv::Mat &gray8u, int shrink, int timeout_ms) {
    DmtxImage *dmtx_img =
        dmtxImageCreate(gray8u.data, gray8u.cols, gray8u.rows, DmtxPack8bppK);
    if (!dmtx_img)
        return "";

    DmtxDecode *dec = dmtxDecodeCreate(dmtx_img, shrink);
    if (!dec) {
        dmtxImageDestroy(&dmtx_img);
        return "";
    }

    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), timeout_ms);
    DmtxRegion *reg = dmtxRegionFindNext(dec, &timeout);

    std::string result;
    if (reg) {
        DmtxMessage *msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
        if (msg) {
            result = std::string(
                reinterpret_cast<char *>(msg->output), msg->outputIdx);
            dmtxMessageDestroy(&msg);
        }
        dmtxRegionDestroy(&reg);
    }
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&dmtx_img);
    return result;
}

// Decode a data matrix from an 8-bit grayscale image. Returns empty string
// on failure. Tries Otsu-binarized and raw inputs across several shrink
// factors, since a single libdmtx pass often misses matrices that dominate
// the frame or have uneven illumination.
std::string read_data_matrix(const cv::Mat &gray8u) {
    cv::Mat binary;
    cv::threshold(gray8u, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    const std::vector<int> shrink_factors = {2, 1, 4};
    const int per_attempt_timeout_ms = 1000;
    const std::vector<std::pair<const char *, const cv::Mat *>> inputs = {
        {"otsu", &binary}, {"raw", &gray8u}};
    for (const auto &[label, img] : inputs) {
        for (int shrink : shrink_factors) {
            spdlog::debug("dmtx attempt: input={}, shrink={}", label, shrink);
            std::string result =
                try_decode_data_matrix(*img, shrink, per_attempt_timeout_ms);
            if (!result.empty()) {
                spdlog::info(
                    "Data matrix decoded (input={}, shrink={})", label, shrink);
                return result;
            }
        }
    }
    return "";
}

// Block until a frame with a received_time different from after_time arrives.
FrameData wait_for_next_frame(
    const std::shared_ptr<LatestFrame>& latest_frame_holder, uint64_t after_time) {
    FrameData frame;
    do {
        frame = latest_frame_holder->get_latest_frame_data();
        if (frame.received_time != after_time)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (true);
    return frame;
}

// Per-axis sign between stage and arena coords: stage_pos = sign * arena_pos +
// offset. The arena is mounted face-down, which mirrors the X axis but leaves
// Y unchanged.
constexpr int arena_x_sign = -1;
constexpr int arena_y_sign = 1;

// Phase 1: live preview with crosshairs; wait for the user to centre the
// camera on the data matrix and press ENTER. On ENTER, returns true and the
// captured raw (un-reoriented) frame in rawFrameOut. On ESC, returns false so
// the caller can shut down and exit.
bool live_alignment_preview(
    const RecorderConfig &recorder_config, cv::Mat &raw_frame_out) {
    cv::namedWindow("Behavior Camera", cv::WINDOW_NORMAL);
    // After reorientation the displayed dimensions are swapped relative to
    // sensor
    int roi_width =
        recorder_config.get_parameter<int>("behavior_camera", "roi_width");
    int roi_height =
        recorder_config.get_parameter<int>("behavior_camera", "roi_height");
    cv::resizeWindow("Behavior Camera", roi_height / 2, roi_width / 2);

    spdlog::info(
        "Live preview started. Move stages so camera is centered on the "
        "data matrix, then press ENTER.");

    while (true) {
        FrameData frame_data = behavior_recording_state->latest_frame_holder
                                   ->get_latest_frame_data();
        if (frame_data.image.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Mat oriented;
        reorient_behavior_image(frame_data.image, oriented);

        cv::Mat display;
        cv::cvtColor(oriented, display, cv::COLOR_GRAY2BGR);

        // Crosshairs at image center
        int cx = display.cols / 2;
        int cy = display.rows / 2;
        cv::Scalar red(0, 0, 255);
        cv::line(
            display, cv::Point(cx, 0), cv::Point(cx, display.rows - 1), red, 1);
        cv::line(
            display, cv::Point(0, cy), cv::Point(display.cols - 1, cy), red, 1);

        cv::putText(
            display,
            "Center camera on data matrix, then press ENTER",
            cv::Point(24, 66),
            cv::FONT_HERSHEY_SIMPLEX,
            1.5,
            red,
            3);

        // Downsample before imshow so the display pipeline isn't saturated by
        // full-resolution frames at the loop rate (caused a session lockup).
        cv::Mat display_small;
        cv::resize(display, display_small, {}, 0.5, 0.5);
        cv::imshow("Behavior Camera", display_small);
        int key = cv::waitKey(33);
        if (key == 13 || key == 10) // ENTER
        {
            // Capture the raw (un-reoriented) frame so dmtx sees the data
            // matrix in its un-mirrored orientation. The display has been
            // horizontally flipped to look natural to the user, but that
            // flip would mirror the data matrix and make it unreadable.
            raw_frame_out = frame_data.image.clone();
            return true;
        }
        if (key == 27) // ESC
        {
            spdlog::info("ESC pressed. Exiting.");
            return false;
        }
    }
}

// Phases 2 & 3: decode the data matrix from the captured frame and verify its
// content against the checksum in metadata.yaml. On failure, logs the error and
// returns the distinguishing result (the caller is responsible for shutdown +
// throw).
enum class ChecksumResult { success, no_data_matrix, checksum_mismatch };

ChecksumResult decode_and_verify_checksum(
    const cv::Mat &raw_frame, const YAML::Node &metadata) {
    spdlog::info("Decoding data matrix...");
    std::string dm_content = read_data_matrix(raw_frame);
    if (dm_content.empty()) {
        spdlog::error("No data matrix found in the captured frame.");
        return ChecksumResult::no_data_matrix;
    }
    spdlog::info("Data matrix decoded: '{}'", dm_content);

    std::string expected_checksum = metadata["checksum"].as<std::string>();
    if (dm_content != expected_checksum) {
        spdlog::error(
            "Checksum mismatch: data matrix='{}', expected='{}'",
            dm_content,
            expected_checksum);
        return ChecksumResult::checksum_mismatch;
    }
    spdlog::info("Checksum verified: '{}'", dm_content);
    return ChecksumResult::success;
}

// Phase 4: compute the offset mapping arena coords to stage coords, from the
// current stage position (centered on the data matrix) and the known data
// matrix center in arena coords.
void compute_stage_to_arena_offset(
    MotionControl &motion_control,
    const YAML::Node &metadata,
    double &offset_x_out,
    double &offset_y_out) {
    double current_x = motion_control.get_position(x_axis);
    double current_y = motion_control.get_position(y_axis);
    spdlog::info(
        "Current stage position when centered on data matrix: ({:.4f}, {:.4f})",
        current_x,
        current_y);

    auto dm_center_vec =
        metadata["datamatrix_pos"]["center"].as<std::vector<double>>();
    double dm_center_x = dm_center_vec[0];
    double dm_center_y = dm_center_vec[1];
    offset_x_out = current_x - arena_x_sign * dm_center_x;
    offset_y_out = current_y - arena_y_sign * dm_center_y;
    spdlog::info(
        "Data matrix center in arena coords: ({:.4f}, {:.4f})",
        dm_center_x,
        dm_center_y);
    spdlog::info(
        "Axis signs (arena -> stage): x_sign={}, y_sign={}",
        arena_x_sign,
        arena_y_sign);
    spdlog::info(
        "Offset (arena -> stage): ({:.4f}, {:.4f})",
        offset_x_out,
        offset_y_out);
    spdlog::info(
        "Arena origin (0,0) maps to stage position ({:.4f}, {:.4f})",
        offset_x_out,
        offset_y_out);
}

// Live preview overlay during the apriltag visit; red dot = shutter.
void show_apriltag_status(int tag_id, bool shutter) {
    FrameData fd =
        behavior_recording_state->latest_frame_holder->get_latest_frame_data();
    if (fd.image.empty())
        return;
    cv::Mat oriented, display;
    reorient_behavior_image(fd.image, oriented);
    cv::cvtColor(oriented, display, cv::COLOR_GRAY2BGR);
    cv::Scalar red(0, 0, 255);
    int cx = display.cols / 2, cy = display.rows / 2;
    cv::line(display, {cx, 0}, {cx, display.rows - 1}, red, 1);
    cv::line(display, {0, cy}, {display.cols - 1, cy}, red, 1);
    cv::putText(
        display,
        fmt::format(
            "{} AprilTag #{}", shutter ? "Capturing" : "Moving to", tag_id),
        {24, 66},
        cv::FONT_HERSHEY_SIMPLEX,
        1.5,
        red,
        3);
    if (shutter)
        cv::circle(display, {display.cols - 40, 40}, 20, red, -1);
    cv::Mat display_small;
    cv::resize(display, display_small, {}, 0.5, 0.5);
    cv::imshow("Behavior Camera", display_small);
    cv::waitKey(1);
}

// Phase 5: visit each AprilTag in id order, drop settling frames, acquire 10
// consecutive frames, and save images + CSV under <arena_dir>/mapping_scan.
// The on_out_of_range callback is invoked (then this rethrows) if a target
// stage position is outside the physical range of motion, so the caller can
// shut down before the exception propagates.
void scan_all_apriltags(
    MotionControl &motion_control,
    const YAML::Node &metadata,
    double offset_x,
    double offset_y,
    double motion_velocity,
    int settling_frames,
    const std::filesystem::path &arena_dir,
    const std::function<void()> &on_out_of_range) {
    YAML::Node apriltag_positions = metadata["apriltag_positions"];

    // Sort by integer key so we visit in a defined order
    std::map<int, YAML::Node> sorted_tags;
    for (auto it = apriltag_positions.begin(); it != apriltag_positions.end();
         ++it)
        sorted_tags[it->first.as<int>()] = it->second;

    std::filesystem::path scan_dir = arena_dir / "mapping_scan";
    std::filesystem::create_directories(scan_dir);
    std::filesystem::path csv_path = scan_dir / "apriltag_stage_positions.csv";
    std::ofstream csv_file(csv_path.string());
    csv_file << "apriltag_id,image_id,stage_x_mm,stage_y_mm\n";

    std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 100};

    for (auto &[tag_id, tag_node] : sorted_tags) {
        auto tag_center = tag_node["center"].as<std::vector<double>>();
        double target_x = arena_x_sign * tag_center[0] + offset_x;
        double target_y = arena_y_sign * tag_center[1] + offset_y;

        spdlog::info(
            "AprilTag {}: arena ({:.4f}, {:.4f}) -> stage ({:.4f}, {:.4f})",
            tag_id,
            tag_center[0],
            tag_center[1],
            target_x,
            target_y);

        auto safe_move_absolute = [&](MotionAxis axis,
                                      const char *axis_name,
                                      double target) {
            try {
                motion_control.move_absolute(
                    axis, target, false, motion_velocity);
            } catch (const zaber::motion::exceptions::BadDataException &) {
                spdlog::error(
                    "Arena placed outside physical range of motion of linear "
                    "stages. Axis: {}, target: {:.4f} mm",
                    axis_name,
                    target);
                on_out_of_range();
                throw std::runtime_error(
                    "Target stage position out of physical range");
            }
        };

        safe_move_absolute(x_axis, "X", target_x);
        safe_move_absolute(y_axis, "Y", target_y);

        while (!motion_control.check_if_idle(x_axis) ||
               !motion_control.check_if_idle(y_axis)) {
            show_apriltag_status(tag_id, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Drop N settling frames (may have been exposed while stages were
        // still settling, or while mechanical vibration was decaying).
        for (int i = 0; i < settling_frames; i++) {
            uint64_t last_time = behavior_recording_state->latest_frame_holder
                                     ->get_latest_frame_data()
                                     .received_time;
            wait_for_next_frame(
                behavior_recording_state->latest_frame_holder, last_time);
            show_apriltag_status(tag_id, false);
        }
        spdlog::debug(
            "AprilTag {}: dropped {} settling frames", tag_id, settling_frames);

        // Acquire 10 consecutive frames
        for (int img_id = 0; img_id < 10; img_id++) {
            uint64_t last_time = behavior_recording_state->latest_frame_holder
                                     ->get_latest_frame_data()
                                     .received_time;
            FrameData frame_data = wait_for_next_frame(
                behavior_recording_state->latest_frame_holder, last_time);

            cv::Mat image;
            reorient_behavior_image(frame_data.image, image);

            std::string filename =
                fmt::format("apriltag{}_img{}.jpg", tag_id, img_id);
            cv::imwrite((scan_dir / filename).string(), image, jpeg_params);
            show_apriltag_status(tag_id, true);

            double pos_x = motion_control.get_position(x_axis);
            double pos_y = motion_control.get_position(y_axis);
            csv_file << tag_id << "," << img_id << ","
                     << fmt::format("{:.6f}", pos_x) << ","
                     << fmt::format("{:.6f}", pos_y) << "\n";

            spdlog::debug(
                "Saved {} at stage ({:.4f}, {:.4f})", filename, pos_x, pos_y);
        }
        spdlog::info("AprilTag {} done: 10 frames saved", tag_id);
    }

    csv_file.close();
    spdlog::info("Stage positions written to {}", csv_path.string());
}
} // namespace

void run_arena_registration_scan(
    const std::filesystem::path &profile_dir,
    const std::filesystem::path &arena_dir) {
    std::filesystem::path metadata_path = arena_dir / "metadata.yaml";

    if (!std::filesystem::exists(arena_dir)) {
        spdlog::error("Arena directory does not exist: {}", arena_dir.string());
        throw std::runtime_error(
            "Arena directory not found: " + arena_dir.string());
    }
    if (!std::filesystem::exists(metadata_path)) {
        spdlog::error(
            "Arena metadata file not found: {}", metadata_path.string());
        throw std::runtime_error(
            "metadata.yaml not found: " + metadata_path.string());
    }

    // Load recorder config
    std::filesystem::path config_path = profile_dir / "recorder_config.yaml";
    spdlog::info(
        "run-arena-registration-scan loading recorder configuration from {}",
        config_path.string());
    RecorderConfig recorder_config(config_path);

    // Set up shared state and start behavior camera acquisition thread
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

    // Wait for camera ready
    size_t retry_count = 0;
    while (!behavior_recording_state->behavior_camera ||
           !behavior_recording_state->behavior_camera->is_ready()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (++retry_count % 20 == 0)
            spdlog::warn("Waiting for behavior camera to initialize...");
    }
    spdlog::info("Behavior camera ready");

    // Start Arduino triggering. There is no muscle camera in the registration
    // scan, so leave muscle imaging off (the default): the behavior camera
    // free-runs. Were it on, the behavior camera would stall waiting for the
    // muscle camera's common-time signal and the live preview would freeze.
    arduino_communication = initialize_triggering_with_default_params(
        recorder_config,
        0, // muscle_num_lines_scanned (ignored since muscle cam not enabled)
        1, // sync ratio (ignored since muscle cam not enabled)
        false); // muscle_imaging_on

    // Set up motion control
    MotionControl motion_control(recorder_config);
    double motion_velocity = recorder_config.get_parameter<double>(
        "motion_control", "default_velocity_mm_per_s");
    int settling_frames = recorder_config.get_parameter<int>(
        "motion_control", "apriltag_mapping_settling_frames");

    auto shutdown = [&]() {
        program_state->to_quit.store(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (behavior_recording_state->behavior_camera) {
            behavior_recording_state->behavior_camera->stop();
            behavior_recording_state->behavior_camera = nullptr;
        }
        if (behavior_thread.joinable())
            behavior_thread.join();
        // The new protocol has no "stop triggering" command; switch the blue
        // excitation light off, then close the link.
        arduino_communication->stop_excitation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arduino_communication->stop_communication();
    };

    // -------------------------------------------------------------------
    // Phase 1: Live preview with crosshairs; wait for user to press ENTER
    // -------------------------------------------------------------------
    cv::Mat raw_frame;
    if (!live_alignment_preview(recorder_config, raw_frame)) {
        shutdown();
        return;
    }

    // -------------------------------------------------------------------
    // Phase 2 & 3: Decode data matrix and verify checksum against metadata
    // -------------------------------------------------------------------
    spdlog::info("Loading arena metadata from {}", metadata_path.string());
    YAML::Node metadata = YAML::LoadFile(metadata_path.string());
    ChecksumResult checksum_result =
        decode_and_verify_checksum(raw_frame, metadata);
    if (checksum_result != ChecksumResult::success) {
        shutdown();
        if (checksum_result == ChecksumResult::no_data_matrix) {
            throw std::runtime_error("No data matrix found");
        } else {
            throw std::runtime_error("Data matrix checksum mismatch");
        }
    }

    // -------------------------------------------------------------------
    // Phase 4: Calculate stage-to-arena offset
    // -------------------------------------------------------------------
    double offset_x, offset_y;
    compute_stage_to_arena_offset(motion_control, metadata, offset_x, offset_y);

    // -------------------------------------------------------------------
    // Phase 5: Visit each AprilTag, acquire 10 frames, save images + CSV
    // -------------------------------------------------------------------
    scan_all_apriltags(
        motion_control,
        metadata,
        offset_x,
        offset_y,
        motion_velocity,
        settling_frames,
        arena_dir,
        shutdown);

    // -------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------
    shutdown();
}

int main(int argc, char **argv) {
    std::signal(SIGINT, [](int) { quit_program(); });

    // Parse CLI
    std::string profile_dir_str = "~/Spotlight/default/";
    std::string arena_dir_str;
    spdlog::level::level_enum log_level = spdlog::level::info;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            // clang-format off
            std::cout
                << "Usage: " << argv[0]
                << " -p PROFILE_DIR -a ARENA_DIR [OPTIONS]\n"
                << "  -p, --profile-dir PATH  Profile directory\n"
                << "  -a, --arena PATH        Arena directory (containing metadata.yaml)\n"
                << "  -v, --verbose           Debug-level logging\n"
                << "  --verbosity LEVEL       trace/debug/info/warn/error/critical/off\n";
            // clang-format on
            return 0;
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc)
            profile_dir_str = argv[++i];
        else if ((arg == "-a" || arg == "--arena") && i + 1 < argc)
            arena_dir_str = argv[++i];
        else if (arg == "-v" || arg == "--verbose")
            log_level = spdlog::level::debug;
        else if (arg == "--verbosity" && i + 1 < argc) {
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

    if (arena_dir_str.empty()) {
        std::cerr << "Error: -a/--arena is required\n";
        return 1;
    }

    spdlog::set_level(log_level);

    std::filesystem::path profile_dir(expand_path(profile_dir_str));
    std::filesystem::path arena_dir(expand_path(arena_dir_str));
    run_arena_registration_scan(profile_dir, arena_dir);
    spdlog::info("Arena registration complete");

    return 0;
}
