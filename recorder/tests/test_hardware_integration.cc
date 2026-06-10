// Hardware integration test: initialize all hardware-control objects and
// threads together, run briefly, then execute the full shutdown sequence.
//
// Covers the same peripherals as the individual hardware tests (behavior
// camera, motion stages, trigger controller, muscle camera) but exercises
// them simultaneously, including the motionControlRequestHandler thread that
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
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/muscle_camera.h"
#include "recorder/common/utils.h"
#include "test_hardware_helpers.h"

namespace {
// How long to let all hardware run before triggering shutdown.
constexpr int kRunDurationSeconds = 3;
// Timeout for waiting for the motion-control handler to report ready.
constexpr int kHandlerReadyTimeoutSeconds = 15;
} // namespace

TEST(HardwareIntegrationTest, AllHardwareInitAndShutoff) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profileDir);
    RecorderConfig config = loadRecorderConfig(profileDir);

    // -------------------------------------------------------------------------
    // Shared program state
    auto programState = std::make_shared<ProgramState>();
    auto trackingControlState = std::make_shared<TrackingControlState>();

    // -------------------------------------------------------------------------
    // Trigger controller: starts its background serial-communication thread
    // in the constructor.
    const std::string arduinoPort = findArduinoPortName(config);
    ASSERT_FALSE(arduinoPort.empty()) << "Arduino not found on any serial port";
    ArduinoCommunication arduino(arduinoPort);

    // -------------------------------------------------------------------------
    // Behavior camera: configure in constructor, then arm for acquisition.
    const int behFullW = config.getParameter<int>("behavior_camera", "full_frame_width");
    const int behFullH = config.getParameter<int>("behavior_camera", "full_frame_height");
    const int behRoiW  = roundToNearestValidBehaviorCamDimension(
        config.getParameter<int>("behavior_camera", "roi_width"));
    const int behRoiH  = roundToNearestValidBehaviorCamDimension(
        config.getParameter<int>("behavior_camera", "roi_height"));
    const std::string ioLine =
        config.getParameter<std::string>("behavior_camera", "frame_grabber_trigger_line");
    auto [behXOffset, behYOffset] =
        getCenteredOffsets(behRoiW, behRoiH, behFullW, behFullH);

    BehaviorCamera behaviorCamera(behRoiW, behRoiH, behXOffset, behYOffset, ioLine);
    EXPECT_TRUE(behaviorCamera.isReady());
    behaviorCamera.start();

    // -------------------------------------------------------------------------
    // Muscle camera: spawns pco-camera-server as a child process.
    const int musFullW = config.getParameter<int>("muscle_camera", "full_frame_width");
    const int musFullH = config.getParameter<int>("muscle_camera", "full_frame_height");
    const int musRoiW  = roundToNearestValidMuscleCamHorizontal(
        config.getParameter<int>("muscle_camera", "roi_width"));
    const int musRoiH  = roundToNearestValidMuscleCamVertical(
        config.getParameter<int>("muscle_camera", "roi_height"));
    const int musXOffset = (musFullW - musRoiW) / 2;
    const int musYOffset = (musFullH - musRoiH) / 2;
    const double lineTimeUs =
        config.getParameter<double>("muscle_camera", "rolling_shutter_line_time_us");
    const double readoutTimeUs =
        config.getParameter<double>("muscle_camera", "sensor_readout_time_us");

    MuscleCamera muscleCamera(
        musRoiW, musRoiH, musXOffset, musYOffset,
        lineTimeUs, readoutTimeUs,
        config, profileDir, spdlog::level::info);
    EXPECT_GT(muscleCamera.getCameraServerPID(), 0);

    // -------------------------------------------------------------------------
    // Motion-control request handler thread: creates MotionControl internally
    // and processes position requests from the global API.
    std::thread motionThread(
        motionControlRequestHandler, std::cref(config),
        trackingControlState, programState);

    // Wait for the handler to signal it is ready (Zaber connection established).
    bool handlerReady = false;
    for (int elapsed = 0;
         elapsed < kHandlerReadyTimeoutSeconds * 10 && !handlerReady;
         ++elapsed) {
        if (trackingControlState->motionControlHandlerReady.load()) {
            handlerReady = true;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    ASSERT_TRUE(handlerReady)
        << "motionControlRequestHandler did not become ready within "
        << kHandlerReadyTimeoutSeconds << " s";

    // -------------------------------------------------------------------------
    // Run briefly to let everything stabilize, then shut down in order.
    std::this_thread::sleep_for(std::chrono::seconds(kRunDurationSeconds));

    // Shutdown: set the quit flag, wake the motion thread, then stop all
    // peripherals in reverse init order.
    programState->toQuit.store(true);
    stopMotionControlRequestHandler(programState);
    motionThread.join();

    arduino.stopCommunication();
    behaviorCamera.stop();
    muscleCamera.stop();

    // All destructors run here.
}
