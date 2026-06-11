// Hardware tests: PCO muscle camera init-configure-destroy cycles.
//
// Two tests:
//   1. Standalone pco-camera-server: spawns the binary directly, verifies it
//      stays alive through its startup sequence, then stops it with SIGTERM
//      and checks for a clean exit.
//   2. MuscleCamera class: constructs the in-process API object (which spawns
//      the server internally), verifies the server is alive after settling,
//      then calls stop().
//
// wait_for_one_frame() is intentionally not called: it uses pthread_cond_wait
// with no timeout, so it would block indefinitely if the server crashed or
// never produced a frame.  The alive-after-N-seconds check is a good proxy
// for "server initialised and is running its acquisition loop."
//
// Requires the PCO panda camera to be powered and connected.
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <cerrno>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/peripherals/muscle_camera.h"
#include "test_hardware_helpers.h"

namespace {
// The test binary is placed in build/tests/; pco-camera-server is in build/.
std::filesystem::path pco_camera_server_path() {
    return std::filesystem::canonical("/proc/self/exe")
               .parent_path() // build/tests/
               .parent_path() // build/
           / "pco-camera-server";
}

// Smallest PCO ROI: width must be a multiple of 32 (>= 64), height a
// multiple of 8 (>= 16).
constexpr unsigned int min_roi_x0 = 1;
constexpr unsigned int min_roi_x1 = 64;
constexpr unsigned int min_roi_y0 = 1;
constexpr unsigned int min_roi_y1 = 16;

// Grace period before SIGKILL after SIGTERM, and the startup wait.
constexpr int grace_period_ms = 5000;
constexpr int poll_interval_ms = 50;
constexpr int startup_wait_seconds = 8;
// How long to let the MuscleCamera class's server settle before the alive
// check.
constexpr int settle_wait_seconds = 5;
} // namespace

// --- Test 1: standalone pco-camera-server binary ----------------------------

TEST(MuscleCameraHardwareTest, StandalonePcoCameraServerInitDestroy) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);

    const std::filesystem::path server_path = pco_camera_server_path();
    ASSERT_TRUE(std::filesystem::exists(server_path))
        << "pco-camera-server not found at " << server_path;

    pid_t pid = fork();
    ASSERT_GE(pid, 0) << "fork() failed: " << strerror(errno);

    if (pid == 0) {
        // Child: exec the server with the minimum valid ROI.
        execl(
            server_path.c_str(),
            "pco-camera-server",
            "--profile-dir",
            profile_dir.c_str(),
            "--x-min",
            std::to_string(min_roi_x0).c_str(),
            "--x-max",
            std::to_string(min_roi_x1).c_str(),
            "--y-min",
            std::to_string(min_roi_y0).c_str(),
            "--y-max",
            std::to_string(min_roi_y1).c_str(),
            static_cast<char *>(nullptr));
        _exit(EXIT_FAILURE); // execl returned → failure
    }

    // Parent: wait for the server to initialise and enter its acquisition loop,
    // then verify it has not crashed.
    std::this_thread::sleep_for(std::chrono::seconds(startup_wait_seconds));
    EXPECT_EQ(kill(pid, 0), 0)
        << "pco-camera-server exited prematurely (PID " << pid << ")";

    // Stop it gracefully.
    kill(pid, SIGTERM);

    bool reaped = false;
    for (int elapsed = 0; elapsed < grace_period_ms;
         elapsed += poll_interval_ms) {
        pid_t result = waitpid(pid, nullptr, WNOHANG);
        if (result == pid || (result == -1 && errno == ECHILD)) {
            reaped = true;
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(poll_interval_ms));
    }

    if (!reaped) {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        FAIL() << "pco-camera-server did not exit within " << grace_period_ms
               << " ms of SIGTERM";
    }
}

// --- Test 2: MuscleCamera class ---------------------------------------------

TEST(MuscleCameraHardwareTest, MuscleCameraClassInitDestroyMuscleCamera) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);
    RecorderConfig config = load_recorder_config(profile_dir);

    const int full_frame_width =
        config.get_parameter<int>("muscle_camera", "full_frame_width");
    const int full_frame_height =
        config.get_parameter<int>("muscle_camera", "full_frame_height");
    const int roi_width = round_to_nearest_valid_muscle_cam_horizontal(
        config.get_parameter<int>("muscle_camera", "roi_width"));
    const int roi_height = round_to_nearest_valid_muscle_cam_vertical(
        config.get_parameter<int>("muscle_camera", "roi_height"));
    // Center the ROI on the sensor.
    const int x_offset = (full_frame_width - roi_width) / 2;
    const int y_offset = (full_frame_height - roi_height) / 2;
    const double line_time_us = config.get_parameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    const double readout_time_us =
        config.get_parameter<double>("muscle_camera", "sensor_readout_time_us");

    // Construction spawns pco-camera-server and sets up shared memory.
    MuscleCamera muscle_camera(
        roi_width,
        roi_height,
        x_offset,
        y_offset,
        line_time_us,
        readout_time_us,
        config,
        profile_dir,
        spdlog::level::info);

    const pid_t server_pid = muscle_camera.get_camera_server_pid();
    ASSERT_GT(server_pid, 0);

    // Let the server settle into its acquisition loop, then confirm it is
    // still alive (has not crashed during initialization).
    std::this_thread::sleep_for(std::chrono::seconds(settle_wait_seconds));
    EXPECT_EQ(kill(server_pid, 0), 0)
        << "pco-camera-server (PID " << server_pid << ") crashed after init";

    // stop() sends SIGTERM to the server and reaps the process.
    muscle_camera.stop();

    // Destructor is a no-op after stop().
}
