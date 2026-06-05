#include <gtest/gtest.h>

#include "recorder/peripherals/motion_control.h"
#include "recorder/common/utils.h"

const float positionTolerance = 0.001;

TEST(TestMotionStages, ConfigureMotionStages) {
    MotionControl motionControl;
}

TEST(TestMotionStages, Homing) {
    MotionControl motionControl;
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 0.0, positionTolerance);
    ASSERT_NEAR(yPosition, 0.0, positionTolerance);
}

TEST(TestMotionStages, MoveAbsolute) {
    MotionControl motionControl;
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    motionControl.moveAbsolute(X_AXIS, 10.0);
    motionControl.moveAbsolute(Y_AXIS, 20.0);
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 10.0, positionTolerance);
    ASSERT_NEAR(yPosition, 20.0, positionTolerance);
}

TEST(TestMotionStages, MoveRelative) {
    MotionControl motionControl;
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

TEST(TestMotionStages, WaitOrNotWait) {
    MotionControl motionControl;
    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);

    uint64_t startTime, endTime;
    double positionAfterMoveCall;

    // With waiting (sync, default)
    startTime = getCurrentTimeMicroseconds();
    motionControl.moveAbsolute(X_AXIS, 10.0, true);
    endTime = getCurrentTimeMicroseconds();
    positionAfterMoveCall = motionControl.getPosition(X_AXIS);
    ASSERT_GT(endTime - startTime, 500000) // should block for >0.5s
        << "moveAbsolute should block until target position is reached "
           "when moving in sync mode (wait=true)";
    ASSERT_NEAR(positionAfterMoveCall, 10.0, positionTolerance)
        << "Position after move call should already be at target value "
           "when moving in sync mode (wait=true)";

    startTime = getCurrentTimeMicroseconds();
    motionControl.moveAbsolute(Y_AXIS, 10.0, false);
    endTime = getCurrentTimeMicroseconds();
    positionAfterMoveCall = motionControl.getPosition(Y_AXIS);
    ASSERT_LT(endTime - startTime, 5000) // should exit within 5ms
        << "moveAbsolute should exit very quickly (< 1us)"
           "when moving in async mode (wait=false)";
    ASSERT_GT(abs(positionAfterMoveCall - 10.0), 9.0)
        << "Position after moveAbsolute call should not be at target value "
           "yet when moving in async mode (wait=false)";
}

TEST(TestMotionStages, MoveBothSyncAndAsync) {
    MotionControl motionControl;
    uint64_t startTime, endTime;

    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    startTime = getCurrentTimeMicroseconds();
    motionControl.moveAbsolute(X_AXIS, 5.0, true);
    motionControl.moveAbsolute(Y_AXIS, 5.0, true);
    endTime = getCurrentTimeMicroseconds();
    float syncDuration = endTime - startTime;

    motionControl.home(X_AXIS);
    motionControl.home(Y_AXIS);
    startTime = getCurrentTimeMicroseconds();
    motionControl.moveAbsolute(X_AXIS, 5.0, false);
    motionControl.moveAbsolute(Y_AXIS, 5.0, false);
    motionControl.waitUntilIdle(X_AXIS);
    motionControl.waitUntilIdle(Y_AXIS);
    endTime = getCurrentTimeMicroseconds();
    float asyncDuration = endTime - startTime;

    ASSERT_NEAR(asyncDuration / syncDuration, 0.5, 0.01)
        << "Moving two stages asynchronously should take half the time of "
           "moving them synchronously";

    // Check that indeed the stages are at the right positions
    double xPosition = motionControl.getPosition(X_AXIS);
    double yPosition = motionControl.getPosition(Y_AXIS);
    ASSERT_NEAR(xPosition, 5.0, positionTolerance)
        << "X axis should be at target position when it becomes idle";
    ASSERT_NEAR(yPosition, 5.0, positionTolerance)
        << "Y axis should be at target position when it becomes idle";
}