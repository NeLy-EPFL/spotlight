/**
 * run-spotlight-headless -- run-spotlight without the GUI, for automated
 * testing and performance analysis.
 *
 * Starts the same Recorder as run-spotlight, records once with the given
 * parameters, and quits once the recording has been handed to the savers:
 *   1. Stream (live, not saving) for --stream seconds.
 *   2. Record into --save-dir, which must not exist or be empty. A scheduled
 *      recording (the parameters' experiment_protocol ends in "<n>/x/stop")
 *      runs to its programmed end; an open one (empty protocol) runs for
 *      --duration seconds.
 * The recording parameters are read from --params, a file in the format of a
 * recording's metadata/experiment_parameters.yaml (so a previous recording can
 * be replayed). Keys that are absent, or the whole file if --params is not
 * given, take the recorder config's defaults, as in the GUI. Only the keys the
 * GUI exposes are read; derived ones such as muscle_nominal_exposure_us are
 * recomputed.
 *
 * Ctrl-C ends the recording (if any) and quits.
 *
 * CLI:  run-spotlight-headless -p PROFILE_DIR -a ARENA_DIR --save-dir DIR
 *           [--params FILE] [--duration SECONDS] [--stream SECONDS] [OPTIONS]
 *       (see --help for details)
 */
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/cli.h"
#include "recorder/common/recorder.h"
#include "recorder/common/utils.h"

namespace {
volatile std::sig_atomic_t sigint_received = 0;

// How long to wait for the behavior camera to come up after startup
constexpr auto behavior_camera_timeout = std::chrono::seconds(60);
// Polling interval for the programmed stop, the duration, and Ctrl-C
constexpr auto poll_interval = std::chrono::milliseconds(100);

// Recording parameters from an experiment_parameters.yaml file; absent keys
// keep their value in `params`. Also reads whether muscle is imaged.
void load_recording_params(
    const std::filesystem::path &params_file,
    RecordingParams &params,
    bool &muscle_imaging_enabled) {
    YAML::Node node = YAML::LoadFile(params_file.string());
    params.behavior_fps =
        node["behavior_fps"].as<int>(params.behavior_fps);
    params.sync_ratio = node["muscle_sync_ratio"].as<int>(params.sync_ratio);
    params.behavior_exposure_time_us =
        node["behavior_exposure_time_us"].as<int>(
            params.behavior_exposure_time_us);
    params.muscle_light_on_time_us = node["muscle_light_on_time_us"].as<int>(
        params.muscle_light_on_time_us);
    params.experiment_protocol = node["experiment_protocol"].as<std::string>(
        params.experiment_protocol);
    muscle_imaging_enabled =
        node["muscle_imaging_enabled"].as<bool>(muscle_imaging_enabled);
}

// Sleep for `duration`, waking up early on Ctrl-C. Returns false on Ctrl-C.
bool sleep_unless_interrupted(std::chrono::duration<double> duration) {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
        if (sigint_received) {
            return false;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return !sigint_received;
}
} // namespace

int run_spotlight_headless_main(int argc, char **argv) {
    std::signal(SIGINT, [](int) { sigint_received = 1; });

    CLIOptions options = parse_cli(argc, argv);
    if (options.arena_dir.empty() || options.save_dir.empty()) {
        std::cerr << "Error: -a/--arena and --save-dir are required.\n";
        return 1;
    }
    spdlog::set_level(options.log_level);

    // Qt serial port classes (used by the Arduino interface) expect a Qt
    // application object, as in run-spotlight; no event loop is run.
    QCoreApplication application(argc, argv);

    Recorder recorder(
        expand_path(options.profile_dir),
        expand_path(options.arena_dir),
        options.no_muscle);

    RecordingParams params = recorder.get_default_recording_params();
    bool muscle_imaging_enabled = false;
    if (!options.params_file.empty()) {
        load_recording_params(
            expand_path(options.params_file), params, muscle_imaging_enabled);
    }
    std::deque<OperationStep> op_sequence;
    bool is_open_recording =
        parse_protocol_string(params.experiment_protocol, op_sequence) == 0;
    if (is_open_recording && options.duration_s <= 0) {
        spdlog::critical("An open recording (empty experiment protocol) needs "
                         "a positive --duration.");
        recorder.quit(1);
    }

    std::filesystem::path save_dir = expand_path(options.save_dir);
    if (std::filesystem::is_directory(save_dir) &&
        !std::filesystem::is_empty(save_dir)) {
        spdlog::critical(
            "Save directory '{}' exists and is not empty.", save_dir.string());
        recorder.quit(1);
    }
    recorder.save_directory()->set_directory(save_dir.string());

    // The behavior camera comes up asynchronously after startup
    auto camera_deadline =
        std::chrono::steady_clock::now() + behavior_camera_timeout;
    while (!recorder.is_behavior_camera_ready()) {
        if (std::chrono::steady_clock::now() > camera_deadline) {
            spdlog::critical("Behavior camera did not become ready.");
            recorder.quit(1);
        }
        if (!sleep_unless_interrupted(poll_interval)) {
            recorder.quit();
        }
    }

    if (muscle_imaging_enabled) {
        recorder.set_muscle_imaging_enabled(true);
    }
    spdlog::info("Streaming for {} s before recording", options.stream_s);
    if (!sleep_unless_interrupted(
            std::chrono::duration<double>(options.stream_s))) {
        recorder.quit();
    }

    if (std::optional<std::string> error = recorder.start_recording(params)) {
        spdlog::critical("Could not start recording: {}", *error);
        recorder.quit(1);
    }
    auto recording_end = std::chrono::steady_clock::now() +
                         std::chrono::duration<double>(options.duration_s);
    while (!recorder.finalize_programmed_stop_if_reached()) {
        bool duration_elapsed =
            is_open_recording &&
            std::chrono::steady_clock::now() >= recording_end;
        if (sigint_received || duration_elapsed) {
            recorder.end_recording(/*reached_programmed_end=*/false);
            break;
        }
        std::this_thread::sleep_for(poll_interval);
    }

    spdlog::info("Waiting for the savers to catch up");
    recorder.wait_until_queues_drained();
    recorder.quit();
}

int main(int argc, char **argv) {
    try {
        return run_spotlight_headless_main(argc, argv);
    } catch (const std::exception &e) {
        spdlog::critical("Fatal error: {}", e.what());
        return 1;
    }
}
