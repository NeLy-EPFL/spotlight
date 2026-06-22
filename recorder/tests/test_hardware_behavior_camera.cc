// Hardware test: BehaviorCamera (Euresys frame grabber) init-configure-destroy
// cycle.  Requires the Euresys grabber and camera to be powered and connected.
//
// The behavior camera is configured for external-trigger mode (trigger line
// from the Arduino trigger controller), so wait_for_one_frame() would block
// indefinitely without the Arduino running.  This test therefore covers only
// the init/configure/start/stop/destroy path; frame acquisition is exercised
// by test_hardware_integration.cc which initializes all peripherals together.
//
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/common/utils.h"
#include "recorder/peripherals/behavior_camera.h"
#include "test_hardware_helpers.h"

TEST(BehaviorCameraHardwareTest, InitConfigureDestroyBehaviorCamera) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);
    RecorderConfig config = load_recorder_config(profile_dir);

    const int full_frame_width =
        config.get_parameter<int>("behavior_camera", "full_frame_width");
    const int full_frame_height =
        config.get_parameter<int>("behavior_camera", "full_frame_height");
    const int roi_width = round_to_nearest_valid_behavior_cam_dimension(
        config.get_parameter<int>("behavior_camera", "roi_width"));
    const int roi_height = round_to_nearest_valid_behavior_cam_dimension(
        config.get_parameter<int>("behavior_camera", "roi_height"));
    const std::string io_line = config.get_parameter<std::string>(
        "behavior_camera", "frame_grabber_trigger_line");

    auto [x_offset, y_offset] = get_centered_offsets(
        roi_width, roi_height, full_frame_width, full_frame_height);

    // Construction configures the grabber and sets the camera-ready flag.
    BehaviorCamera camera(roi_width, roi_height, x_offset, y_offset, io_line);
    EXPECT_TRUE(camera.is_ready());

    // start() arms the grabber for frame acquisition.
    camera.start();

    // stop() disarms the grabber.
    camera.stop();

    // Destructor runs here; is_ready() is cleared in ~BehaviorCamera().
}
