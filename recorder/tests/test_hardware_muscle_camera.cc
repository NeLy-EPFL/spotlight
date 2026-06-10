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
// waitForOneFrame() is intentionally not called: it uses pthread_cond_wait
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
#include <thread>
#include <unistd.h>
#include <sys/wait.h>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/peripherals/muscle_camera.h"
#include "test_hardware_helpers.h"

namespace {
// The test binary is placed in build/tests/; pco-camera-server is in build/.
std::filesystem::path pcoCameraServerPath() {
    return std::filesystem::canonical("/proc/self/exe")
               .parent_path()   // build/tests/
               .parent_path()   // build/
           / "pco-camera-server";
}

// Smallest PCO ROI: width must be a multiple of 32 (>= 64), height a
// multiple of 8 (>= 16).
constexpr unsigned int kMinRoiX0 = 1;
constexpr unsigned int kMinRoiX1 = 64;
constexpr unsigned int kMinRoiY0 = 1;
constexpr unsigned int kMinRoiY1 = 16;

// Grace period before SIGKILL after SIGTERM, and the startup wait.
constexpr int kGracePeriodMs = 5000;
constexpr int kPollIntervalMs = 50;
constexpr int kStartupWaitSeconds = 8;
// How long to let the MuscleCamera class's server settle before the alive check.
constexpr int kSettleWaitSeconds = 5;
} // namespace

// --- Test 1: standalone pco-camera-server binary ----------------------------

TEST(MuscleCameraHardwareTest, StandalonePcoCameraServerInitDestroy) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profileDir);

    const std::filesystem::path serverPath = pcoCameraServerPath();
    ASSERT_TRUE(std::filesystem::exists(serverPath))
        << "pco-camera-server not found at " << serverPath;

    pid_t pid = fork();
    ASSERT_GE(pid, 0) << "fork() failed: " << strerror(errno);

    if (pid == 0) {
        // Child: exec the server with the minimum valid ROI.
        execl(
            serverPath.c_str(),
            "pco-camera-server",
            "--profile-dir", profileDir.c_str(),
            "--x-min", std::to_string(kMinRoiX0).c_str(),
            "--x-max", std::to_string(kMinRoiX1).c_str(),
            "--y-min", std::to_string(kMinRoiY0).c_str(),
            "--y-max", std::to_string(kMinRoiY1).c_str(),
            static_cast<char *>(nullptr));
        _exit(EXIT_FAILURE); // execl returned → failure
    }

    // Parent: wait for the server to initialise and enter its acquisition loop,
    // then verify it has not crashed.
    std::this_thread::sleep_for(std::chrono::seconds(kStartupWaitSeconds));
    EXPECT_EQ(kill(pid, 0), 0)
        << "pco-camera-server exited prematurely (PID " << pid << ")";

    // Stop it gracefully.
    kill(pid, SIGTERM);

    bool reaped = false;
    for (int elapsed = 0; elapsed < kGracePeriodMs; elapsed += kPollIntervalMs) {
        pid_t result = waitpid(pid, nullptr, WNOHANG);
        if (result == pid || (result == -1 && errno == ECHILD)) {
            reaped = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
    }

    if (!reaped) {
        kill(pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        FAIL() << "pco-camera-server did not exit within "
               << kGracePeriodMs << " ms of SIGTERM";
    }
}

// --- Test 2: MuscleCamera class ---------------------------------------------

TEST(MuscleCameraHardwareTest, MuscleCameraClassInitDestroyMuscleCamera) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profileDir);
    RecorderConfig config = loadRecorderConfig(profileDir);

    const int fullFrameWidth =
        config.getParameter<int>("muscle_camera", "full_frame_width");
    const int fullFrameHeight =
        config.getParameter<int>("muscle_camera", "full_frame_height");
    const int roiWidth = roundToNearestValidMuscleCamHorizontal(
        config.getParameter<int>("muscle_camera", "roi_width"));
    const int roiHeight = roundToNearestValidMuscleCamVertical(
        config.getParameter<int>("muscle_camera", "roi_height"));
    // Center the ROI on the sensor.
    const int xOffset = (fullFrameWidth - roiWidth) / 2;
    const int yOffset = (fullFrameHeight - roiHeight) / 2;
    const double lineTimeUs =
        config.getParameter<double>("muscle_camera", "rolling_shutter_line_time_us");
    const double readoutTimeUs =
        config.getParameter<double>("muscle_camera", "sensor_readout_time_us");

    // Construction spawns pco-camera-server and sets up shared memory.
    MuscleCamera muscleCamera(
        roiWidth, roiHeight, xOffset, yOffset,
        lineTimeUs, readoutTimeUs,
        config, profileDir, spdlog::level::info);

    const pid_t serverPid = muscleCamera.getCameraServerPID();
    ASSERT_GT(serverPid, 0);

    // Let the server settle into its acquisition loop, then confirm it is
    // still alive (has not crashed during initialization).
    std::this_thread::sleep_for(std::chrono::seconds(kSettleWaitSeconds));
    EXPECT_EQ(kill(serverPid, 0), 0)
        << "pco-camera-server (PID " << serverPid << ") crashed after init";

    // stop() sends SIGTERM to the server and reaps the process.
    muscleCamera.stop();

    // Destructor is a no-op after stop().
}
