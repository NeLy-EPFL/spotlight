/**
 * run-spotlight -- main recording application.
 *
 * Loads the recorder config from <profile_dir>/recorder_config.yaml, the
 * muscle-camera ROIs from <profile_dir>/muscle_camera_roi.yaml (unless started
 * with --no-muscle, in which case muscle imaging is unavailable), and the arena
 * registration model from <arena_dir>/model/calibration_result.yaml.
 * Reads arena dimensions from <arena_dir>/metadata.yaml to derive the stage
 * range used by the motion-stage preview widget.
 *
 * Launches all acquisition, saving, tracking, and Arduino threads and opens the
 * main GUI window. Whether muscle is imaged, and the muscle/behavior frame
 * rates and exposures, are controlled from the main GUI window at runtime.
 *
 * The recording logic lives in Recorder (common/recorder.h), shared with
 * run-spotlight-headless; this program is its GUI front end.
 *
 * CLI:  run-spotlight -p PROFILE_DIR -a ARENA_DIR [OPTIONS]
 *       (see --help for details; -a/--arena is required)
 */
#include "recorder/apps/run_spotlight.h"

namespace {
// Set by the SIGINT handler. The handler must stay async-signal-safe, so it
// only flips this flag; a QTimer on the GUI thread (see run_spotlight_main)
// polls it and performs the actual teardown via Recorder::quit().
volatile std::sig_atomic_t sigint_received = 0;
} // namespace

int run_spotlight_main(int argc, char **argv) {
    // Keep the handler async-signal-safe: only set a flag. The poll timer
    // installed below runs the real teardown on the GUI thread.
    std::signal(SIGINT, [](int) { sigint_received = 1; });

    // Parse command line arguments
    CLIOptions options = parse_cli(argc, argv);
    if (options.arena_dir.empty()) {
        std::cerr << "Error: -a/--arena is required (path to the arena "
                     "directory containing metadata.yaml and model/).\n";
        return 1;
    }

    // Set log level based on CLI options
    spdlog::set_level(options.log_level);

    QApplication application(argc, argv);

    // Launch all acquisition, saving, tracking, and Arduino threads
    Recorder recorder(
        expand_path(options.profile_dir),
        expand_path(options.arena_dir),
        options.no_muscle);

    // Create and show GUI
    MainGUIWindow local_main_gui_window(recorder);
    local_main_gui_window.show();

    // Poll the SIGINT flag on the GUI thread and tear down when it is set.
    // (The handler itself only sets the flag, to stay async-signal-safe.)
    QTimer sigint_poll_timer;
    QObject::connect(&sigint_poll_timer, &QTimer::timeout, [&recorder]() {
        if (sigint_received) {
            spdlog::info("SIGINT received; shutting down.");
            recorder.quit();
        }
    });
    sigint_poll_timer.start(100); // ms

    // exec() does not return: teardown goes through Recorder::quit() (from the
    // GUI close handler or the SIGINT poll timer above), which std::exit()s.
    return application.exec();
}

int main(int argc, char **argv) {
    try {
        return run_spotlight_main(argc, argv);
    } catch (const std::exception &e) {
        spdlog::critical("Fatal startup error: {}", e.what());
        return 1;
    }
}
