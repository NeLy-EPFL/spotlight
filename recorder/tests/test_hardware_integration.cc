// Hardware integration test: initialize all hardware-control objects and
// threads together, run briefly, then execute the full shutdown sequence.
//
// Covers the same peripherals as the individual hardware tests (behavior
// camera, motion stages, trigger controller, muscle camera) but exercises
// them simultaneously, including the motion_control_request_handler thread that
// the real recorder uses.  Frame acquisition is validated in the individual
// camera tests; this test focuses on the init → run → shutoff cycle.
//
// Requires all four peripherals to be powered and connected.
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/tracking_control.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/muscle_camera.h"
#include "test_hardware_helpers.h"

namespace {
// How long to let all hardware run before triggering shutdown.
constexpr int run_duration_seconds = 3;
// Timeout for waiting for the motion-control handler to report ready.
constexpr int handler_ready_timeout_seconds = 15;
} // namespace

TEST(HardwareIntegrationTest, AllHardwareInitAndShutoff) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);
    RecorderConfig config = load_recorder_config(profile_dir);

    // -------------------------------------------------------------------------
    // Shared program state
    auto program_state = std::make_shared<ProgramState>();
    auto tracking_control_state = std::make_shared<TrackingControlState>();

    // -------------------------------------------------------------------------
    // Trigger controller: starts its background serial-communication thread
    // in the constructor.
    const std::string arduino_port = find_arduino_port_name(config);
    ASSERT_FALSE(arduino_port.empty())
        << "Arduino not found on any serial port";
    ArduinoCommunication arduino(arduino_port);

    // -------------------------------------------------------------------------
    // Behavior camera: configure in constructor, then arm for acquisition.
    const int beh_full_w =
        config.get_parameter<int>("behavior_camera", "full_frame_width");
    const int beh_full_h =
        config.get_parameter<int>("behavior_camera", "full_frame_height");
    const int beh_roi_w = round_to_nearest_valid_behavior_cam_dimension(
        config.get_parameter<int>("behavior_camera", "roi_width"));
    const int beh_roi_h = round_to_nearest_valid_behavior_cam_dimension(
        config.get_parameter<int>("behavior_camera", "roi_height"));
    const std::string io_line = config.get_parameter<std::string>(
        "behavior_camera", "frame_grabber_trigger_line");
    auto [beh_x_offset, beh_y_offset] =
        get_centered_offsets(beh_roi_w, beh_roi_h, beh_full_w, beh_full_h);

    BehaviorCamera behavior_camera(
        beh_roi_w, beh_roi_h, beh_x_offset, beh_y_offset, io_line);
    EXPECT_TRUE(behavior_camera.is_ready());
    behavior_camera.start();

    // -------------------------------------------------------------------------
    // Muscle camera: spawns pco-camera-server as a child process.
    const int mus_full_w =
        config.get_parameter<int>("muscle_camera", "full_frame_width");
    const int mus_full_h =
        config.get_parameter<int>("muscle_camera", "full_frame_height");
    const int mus_roi_w = round_to_nearest_valid_muscle_cam_horizontal(
        config.get_parameter<int>("muscle_camera", "roi_width"));
    const int mus_roi_h = round_to_nearest_valid_muscle_cam_vertical(
        config.get_parameter<int>("muscle_camera", "roi_height"));
    const int mus_x_offset = (mus_full_w - mus_roi_w) / 2;
    const int mus_y_offset = (mus_full_h - mus_roi_h) / 2;
    const double line_time_us = config.get_parameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    const double readout_time_us =
        config.get_parameter<double>("muscle_camera", "sensor_readout_time_us");

    MuscleCamera muscle_camera(
        mus_roi_w,
        mus_roi_h,
        mus_x_offset,
        mus_y_offset,
        line_time_us,
        readout_time_us,
        config,
        profile_dir,
        spdlog::level::info);
    EXPECT_GT(muscle_camera.get_camera_server_pid(), 0);

    // -------------------------------------------------------------------------
    // Motion-control request handler thread: creates MotionControl internally
    // and processes position requests from the global API.
    std::thread motion_thread(
        motion_control_request_handler,
        std::cref(config),
        tracking_control_state,
        program_state);

    // Wait for the handler to signal it is ready (Zaber connection
    // established).
    bool handler_ready = false;
    for (int elapsed = 0;
         elapsed < handler_ready_timeout_seconds * 10 && !handler_ready;
         ++elapsed) {
        if (tracking_control_state->motion_control_handler_ready.load()) {
            handler_ready = true;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    ASSERT_TRUE(handler_ready)
        << "motion_control_request_handler did not become ready within "
        << handler_ready_timeout_seconds << " s";

    // -------------------------------------------------------------------------
    // Run briefly to let everything stabilize, then shut down in order.
    std::this_thread::sleep_for(std::chrono::seconds(run_duration_seconds));

    // Shutdown: set the quit flag, wake the motion thread, then stop all
    // peripherals in reverse init order.
    program_state->to_quit.store(true);
    stop_motion_control_request_handler(program_state);
    motion_thread.join();

    arduino.stop_communication();
    behavior_camera.stop();
    muscle_camera.stop();

    // All destructors run here.
}
