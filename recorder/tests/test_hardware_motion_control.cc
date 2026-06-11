// Hardware test: MotionControl (Zaber XY stages) init-configure-destroy cycle.
// Moves each axis by at most 0.05 mm and returns, keeping total displacement
// within the 0.1 mm safety limit.
// Requires the Zaber stages to be powered and connected.
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/peripherals/motion_control.h"
#include "test_hardware_helpers.h"

namespace {
constexpr double test_move_distance_mm = 0.05;
constexpr double test_velocity_mm_per_s = 1.0;
constexpr double position_tolerance_mm = 0.01;
} // namespace

TEST(MotionControlHardwareTest, InitConfigureDestroyMotionControl) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);
    RecorderConfig config = load_recorder_config(profile_dir);

    // Construction opens the serial port and configures both axes.
    MotionControl motion_control(config);

    // Verify both axes respond to position queries.
    const double initial_x = motion_control.get_position(X_AXIS);
    const double initial_y = motion_control.get_position(Y_AXIS);
    EXPECT_FALSE(std::isnan(initial_x));
    EXPECT_FALSE(std::isnan(initial_y));

    // Move X axis by +0.05 mm and back, staying well within the 0.1 mm limit.
    const double x_min = motion_control.get_min_position(X_AXIS);
    const double x_max = motion_control.get_max_position(X_AXIS);
    ASSERT_GE(initial_x - x_min, test_move_distance_mm)
        << "X axis is too close to the lower limit for a safe test move";
    ASSERT_LE(initial_x + test_move_distance_mm, x_max)
        << "X axis is too close to the upper limit for a safe test move";

    motion_control.move_relative(
        X_AXIS, test_move_distance_mm, /*wait=*/true, test_velocity_mm_per_s);
    const double moved_x = motion_control.get_position(X_AXIS);
    EXPECT_NEAR(
        moved_x, initial_x + test_move_distance_mm, position_tolerance_mm);

    motion_control.move_relative(
        X_AXIS, -test_move_distance_mm, /*wait=*/true, test_velocity_mm_per_s);
    const double restored_x = motion_control.get_position(X_AXIS);
    EXPECT_NEAR(restored_x, initial_x, position_tolerance_mm);

    // Destructor closes the Zaber connection.
}
