// HARDWARE tests that drive the muscle (PCO) camera, and both cameras together.
//
// Unlike test_behavior_camera_hardware.cc (which is PCO-free, to isolate the
// behavior/Euresys path), this file builds into the `recorder_hardware_tests_pco`
// executable, which links BOTH the PCO and Euresys SDKs -- the muscle camera needs
// PCO and the combined test needs both. Like the behavior hardware test, it is
// tagged with the CTest label "hardware" (excluded from the default run; use
// `ctest -L hardware` to run). See docs/setup/building.md.
//
// On failure each test reports the exception's dynamic type and (for Euresys
// exceptions) the GenTL error code/description, so the real cause is visible.

#include "recorder/common/recorder_config.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/muscle_camera.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <tuple>
#include <typeinfo>

#include <EGenTLErrors.h> // Euresys::gentl_error
#include <gtest/gtest.h>

namespace {
// The path to the shipped example profile config is provided by CMake
// (EXAMPLE_CONFIG_PATH); the cameras read their full-frame size, ROI, streaming
// rates, and sensor readout time from it.
RecorderConfig loadExampleConfig() {
    RecorderConfig config{std::string(EXAMPLE_CONFIG_PATH)};
    EXPECT_TRUE(config.isDefined)
        << "Failed to load example config at " << EXAMPLE_CONFIG_PATH;
    return config;
}

// Behavior-camera ROI (centered, dimensions rounded to valid multiples), matching
// what behaviorImageAcquirer derives from the config.
struct BehaviorRoi {
    int width, height, xOffset, yOffset;
};
BehaviorRoi behaviorRoiFromConfig(const RecorderConfig &config) {
    int fullW =
        config.getParameter<int>("behavior_camera", "full_frame_width");
    int fullH =
        config.getParameter<int>("behavior_camera", "full_frame_height");
    int width = roundToNearestValidBehaviorCamDimension(
        config.getParameter<int>("behavior_camera", "roi_width"));
    int height = roundToNearestValidBehaviorCamDimension(
        config.getParameter<int>("behavior_camera", "roi_height"));
    auto [xOffset, yOffset] = getCenteredOffsets(width, height, fullW, fullH);
    return {width, height, xOffset, yOffset};
}

std::unique_ptr<BehaviorCamera> makeBehaviorCamera(const RecorderConfig &config) {
    BehaviorRoi roi = behaviorRoiFromConfig(config);
    std::string ioLine = config.getParameter<std::string>(
        "behavior_camera", "frame_grabber_trigger_line");
    return std::make_unique<BehaviorCamera>(
        static_cast<unsigned int>(roi.width),
        static_cast<unsigned int>(roi.height),
        static_cast<unsigned int>(roi.xOffset),
        static_cast<unsigned int>(roi.yOffset),
        ioLine);
}

// Muscle-camera ROI (centered, dimensions taken from config, offsets rounded to
// valid multiples).
struct MuscleRoi {
    int width, height, xOffset, yOffset;
    double sensorReadoutTimeUs;
};
MuscleRoi muscleRoiFromConfig(const RecorderConfig &config) {
    int fullW = config.getParameter<int>("muscle_camera", "full_frame_width");
    int fullH = config.getParameter<int>("muscle_camera", "full_frame_height");
    int width = config.getParameter<int>("muscle_camera", "roi_width");
    int height = config.getParameter<int>("muscle_camera", "roi_height");
    int xOffset = roundToNearestValidMuscleCamHorizontal((fullW - width) / 2);
    int yOffset = roundToNearestValidMuscleCamVertical((fullH - height) / 2);
    double readout =
        config.getParameter<double>("muscle_camera", "sensor_readout_time_us");
    return {width, height, xOffset, yOffset, readout};
}

std::unique_ptr<MuscleCamera> makeMuscleCamera(const RecorderConfig &config) {
    MuscleRoi roi = muscleRoiFromConfig(config);
    return std::make_unique<MuscleCamera>(
        roi.width,
        roi.height,
        roi.xOffset,
        roi.yOffset,
        roi.sensorReadoutTimeUs,
        config);
}
} // namespace

// Initialize the muscle camera (which opens the PCO device, configures it, and
// starts free-running), confirm it produces a frame, then close it. The watchdog
// makes the frame wait bounded so a non-producing camera fails instead of
// hanging.
TEST(MuscleCameraHardware, InitializesAcquiresAndCloses) {
    RecorderConfig config = loadExampleConfig();

    std::unique_ptr<MuscleCamera> camera;
    try {
        camera = makeMuscleCamera(config);
    } catch (const std::exception &e) {
        FAIL() << "MuscleCamera construction threw [" << typeid(e).name()
               << "]: " << e.what();
    }

    // The PCO camera free-runs (auto-sequence), so frames arrive without external
    // triggering. Bound the wait: if no frame arrives, stop() unblocks the grab.
    std::atomic<bool> gotFrame{false};
    std::thread watchdog([&]() {
        for (int i = 0; i < 50 && !gotFrame.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!gotFrame.load()) {
            camera->stop();
        }
    });
    std::optional<FrameData> frame = camera->waitForOneFrame();
    gotFrame.store(true);
    watchdog.join();

    EXPECT_TRUE(frame.has_value())
        << "Muscle camera produced no frame within ~5 s.";
    if (frame.has_value()) {
        EXPECT_FALSE(frame->image.empty());
    }

    // Close: stop() then destroy. ~MuscleCamera stops the camera and calls
    // PCO_CleanupLib(); this must not throw or hang.
    camera->stop();
    EXPECT_NO_THROW(camera.reset());
}

// Bring up BOTH cameras using the SAME serialized order as align-cameras and
// run-spotlight: the behavior (Euresys) camera is fully initialized first, and
// only then is the muscle (PCO) camera initialized. (Initializing them
// concurrently was observed to hang the Euresys GenTL discovery -- see
// docs/troubleshooting.md.) Here that ordering is enforced by constructing them
// one after the other on this thread rather than via the apps' acquirer threads,
// but the property under test is the same: no overlap between the two SDK inits.
TEST(BothCamerasHardware, SequentialInitBringsBothUp) {
    RecorderConfig config = loadExampleConfig();

    // 1) Behavior camera first, fully (discovery + configure).
    std::unique_ptr<BehaviorCamera> behaviorCamera;
    try {
        behaviorCamera = makeBehaviorCamera(config);
    } catch (const Euresys::gentl_error &e) {
        FAIL() << "BehaviorCamera construction threw Euresys::gentl_error "
               << static_cast<int>(e.gc_err)
               << (e.description.empty() ? "" : " (" + e.description + ")")
               << ": " << e.what();
    } catch (const std::exception &e) {
        FAIL() << "BehaviorCamera construction threw [" << typeid(e).name()
               << "]: " << e.what();
    }
    ASSERT_TRUE(behaviorCamera->isReady())
        << "Behavior camera did not become ready; not starting the muscle "
           "camera.";

    // 2) Only now the muscle camera (PCO init must not overlap the Euresys init
    // above).
    std::unique_ptr<MuscleCamera> muscleCamera;
    try {
        muscleCamera = makeMuscleCamera(config);
    } catch (const std::exception &e) {
        FAIL() << "MuscleCamera construction threw [" << typeid(e).name()
               << "]: " << e.what();
    }

    // Both cameras came up. (Behavior frames need the external trigger, which is
    // not set up here, so we do not grab a behavior frame.) Close both; the
    // destructors must not throw or hang.
    muscleCamera->stop();
    EXPECT_NO_THROW(muscleCamera.reset());
    behaviorCamera->stop();
    EXPECT_NO_THROW(behaviorCamera.reset());
}
