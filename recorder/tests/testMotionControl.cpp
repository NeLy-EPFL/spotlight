#include <gtest/gtest.h>

#include "../src/peripherals/motionControl.hpp"

const float positionTolerance = 0.001;

TEST(TestMotionStages, ConfigureMotionStages)
{
    MotionControl motionControl("/dev/ttyACM0");
}

TEST(TestMotionStages, Homing)
{
    MotionControl motionControl("/dev/ttyACM0");
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 0.0, positionTolerance);
    ASSERT_NEAR(yPosition, 0.0, positionTolerance);
}

TEST(TestMotionStages, MoveAbsolute)
{
    MotionControl motionControl("/dev/ttyACM0");
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    motionControl.moveAbsolute(X_AXIS, 10.0);
    motionControl.moveAbsolute(Y_AXIS, 20.0);
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 10.0, positionTolerance);
    ASSERT_NEAR(yPosition, 20.0, positionTolerance);
}

TEST(TestMotionStages, MoveRelative)
{
    MotionControl motionControl("/dev/ttyACM0");
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    motionControl.moveRelative(X_AXIS, 10.0);
    motionControl.moveRelative(Y_AXIS, 20.0);
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 10.0, positionTolerance);
    ASSERT_NEAR(yPosition, 20.0, positionTolerance);
    motionControl.moveRelative(X_AXIS, -5.0);
    motionControl.moveRelative(Y_AXIS, -10.0);
    xPosition = motionControl.getPosition(X_AXIS);
    yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 10.0 - 5.0, positionTolerance);
    ASSERT_NEAR(yPosition, 20.0 - 10.0, positionTolerance);
}