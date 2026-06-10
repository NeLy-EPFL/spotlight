// Hardware test: BehaviorCamera (Euresys frame grabber) init-configure-destroy
// cycle.  Requires the Euresys grabber and camera to be powered and connected.
//
// The behavior camera is configured for external-trigger mode (trigger line
// from the Arduino trigger controller), so waitForOneFrame() would block
// indefinitely without the Arduino running.  This test therefore covers only
// the init/configure/start/stop/destroy path; frame acquisition is exercised
// by test_hardware_integration.cc which initializes all peripherals together.
//
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/peripherals/behavior_camera.h"
#include "recorder/common/utils.h"
#include "test_hardware_helpers.h"

TEST(BehaviorCameraHardwareTest, InitConfigureDestroyBehaviorCamera) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profileDir);
    RecorderConfig config = loadRecorderConfig(profileDir);

    const int fullFrameWidth =
        config.getParameter<int>("behavior_camera", "full_frame_width");
    const int fullFrameHeight =
        config.getParameter<int>("behavior_camera", "full_frame_height");
    const int roiWidth =
        roundToNearestValidBehaviorCamDimension(
            config.getParameter<int>("behavior_camera", "roi_width"));
    const int roiHeight =
        roundToNearestValidBehaviorCamDimension(
            config.getParameter<int>("behavior_camera", "roi_height"));
    const std::string ioLine =
        config.getParameter<std::string>(
            "behavior_camera", "frame_grabber_trigger_line");

    auto [xOffset, yOffset] =
        getCenteredOffsets(roiWidth, roiHeight, fullFrameWidth, fullFrameHeight);

    // Construction configures the grabber and sets the camera-ready flag.
    BehaviorCamera camera(roiWidth, roiHeight, xOffset, yOffset, ioLine);
    EXPECT_TRUE(camera.isReady());

    // start() arms the grabber for frame acquisition.
    camera.start();

    // stop() disarms the grabber.
    camera.stop();

    // Destructor runs here; isReady() is cleared in ~BehaviorCamera().
}
