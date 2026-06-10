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
constexpr double kTestMoveDistanceMm = 0.05;
constexpr double kTestVelocityMmPerS = 1.0;
constexpr double kPositionToleranceMm = 0.01;
} // namespace

TEST(MotionControlHardwareTest, InitConfigureDestroyMotionControl) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profileDir);
    RecorderConfig config = loadRecorderConfig(profileDir);

    // Construction opens the serial port and configures both axes.
    MotionControl motionControl(config);

    // Verify both axes respond to position queries.
    const double initialX = motionControl.getPosition(X_AXIS);
    const double initialY = motionControl.getPosition(Y_AXIS);
    EXPECT_FALSE(std::isnan(initialX));
    EXPECT_FALSE(std::isnan(initialY));

    // Move X axis by +0.05 mm and back, staying well within the 0.1 mm limit.
    const double xMin = motionControl.getMinPosition(X_AXIS);
    const double xMax = motionControl.getMaxPosition(X_AXIS);
    ASSERT_GE(initialX - xMin, kTestMoveDistanceMm)
        << "X axis is too close to the lower limit for a safe test move";
    ASSERT_LE(initialX + kTestMoveDistanceMm, xMax)
        << "X axis is too close to the upper limit for a safe test move";

    motionControl.moveRelative(
        X_AXIS, kTestMoveDistanceMm, /*wait=*/true, kTestVelocityMmPerS);
    const double movedX = motionControl.getPosition(X_AXIS);
    EXPECT_NEAR(movedX, initialX + kTestMoveDistanceMm, kPositionToleranceMm);

    motionControl.moveRelative(
        X_AXIS, -kTestMoveDistanceMm, /*wait=*/true, kTestVelocityMmPerS);
    const double restoredX = motionControl.getPosition(X_AXIS);
    EXPECT_NEAR(restoredX, initialX, kPositionToleranceMm);

    // Destructor closes the Zaber connection.
}
