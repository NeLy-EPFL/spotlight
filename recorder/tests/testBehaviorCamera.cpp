#include <gtest/gtest.h>

#include <vector>
#include <opencv2/opencv.hpp>

#include "../src/peripherals/behaviorCamera.h"
#include "../src/constants.h"
#include "../src/utils.h"

TEST(RoundingToMultiplesOf64Test, RoundToMultiplesOf64NoOp)
{
    unsigned int value = 640;
    unsigned int roundedValue = roundToMultiplesOf64(value);
    ASSERT_EQ(roundedValue, 640);
}

TEST(RoundingToMultiplesOf64Test, RoundToMultiplesOf64RoundDown)
{
    unsigned int value = 641;
    unsigned int roundedValue = roundToMultiplesOf64(value);
    ASSERT_EQ(roundedValue, 640);
}

TEST(RoundingToMultiplesOf64Test, RoundToMultiplesOf64RoundUp)
{
    unsigned int value = 639;
    unsigned int roundedValue = roundToMultiplesOf64(value);
    ASSERT_EQ(roundedValue, 640);
}

TEST(GetCenteredOffsetsTest, GetCenteredOffsets)
{
    unsigned int xOffset, yOffset;
    std::tie(xOffset, yOffset) = getCenteredOffsets(
        640,
        512,
        BEHAVIOR_CAMERA_FULL_FRAME_WIDTH,
        BEHAVIOR_CAMERA_FULL_FRAME_HEIGHT);
    ASSERT_EQ(xOffset, 960);
    ASSERT_EQ(yOffset, 768);
}

TEST(BehaviorCameraTest, ConfigureBehaviorCamera)
{
    unsigned int imageWidth = roundToMultiplesOf64(640);
    unsigned int imageHeight = roundToMultiplesOf64(480);
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    BehaviorCamera behaviorCamera(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        BEHAVIOR_CAMERA_FRAME_GRABBER_TRIGGER_LINE);
}

TEST(BehaviorCameraTest, BehaviorCameraAcquisition)
/* This test requires the hardware to be configured correctly:
 * An Arduino should ouput a `expectedFrameRate` Hz square wave
 * to the TTLIO12 line of the frame grabber. The width of the
 * squares control the exposure time.
 */
{
    const int expectedFrameRate = 50;
    const int maxPerFrameProcessingTimeAllowedMicroseconds = 100;

    unsigned int imageWidth = roundToMultiplesOf64(640);
    unsigned int imageHeight = roundToMultiplesOf64(480);
    unsigned int xOffset = 0;
    unsigned int yOffset = 0;

    std::vector<FrameData> frameDataVector;
    std::vector<int> blockingTimesMicroseconds;

    BehaviorCamera behaviorCamera(
        imageWidth,
        imageHeight,
        xOffset,
        yOffset,
        BEHAVIOR_CAMERA_FRAME_GRABBER_TRIGGER_LINE);

    int numFrames = expectedFrameRate;
    uint64_t acquisitionStartTime = getCurrentTimeMicroseconds();
    for (int i = 0; i < numFrames; i++)
    {
        double blockingStartTimeMicroseconds = getCurrentTimeMicroseconds();
        FrameData frameData = behaviorCamera.waitForOneFrame();
        double blockingEndTimeMicroseconds = getCurrentTimeMicroseconds();
        frameDataVector.push_back(frameData);
        blockingTimesMicroseconds.push_back(
            blockingEndTimeMicroseconds - blockingStartTimeMicroseconds);
    }
    uint64_t acquisitionEndTime = getCurrentTimeMicroseconds();

    // Check if frame rate is right
    double elapsedTime = acquisitionEndTime - acquisitionStartTime;
    float actualFrameRate = numFrames / elapsedTime;
    ASSERT_NEAR(actualFrameRate, expectedFrameRate, 1)
        << "Expected frame rate: " << expectedFrameRate << " Hz; "
        << "actual frame rate: " << actualFrameRate << " Hz";

    // Check if the frames are different from each other
    for (int i = 0; i < (int) frameDataVector.size() - 1; i++)
    {
        cv::Mat thisImage = *frameDataVector[i].imagePtr;
        cv::Mat nextImage = *frameDataVector[i + 1].imagePtr;
        cv::Mat diffImage;
        cv::absdiff(thisImage, nextImage, diffImage);
        double diffSum = cv::sum(diffImage)[0];
        ASSERT_GT(diffSum, 0)
            << "Frames " << i << " and " << i + 1 << " are identical";
    }

    // Check if acquisition is fast enough
    double blockingTimeSum = 0;
    for (int i = 0; i < (int) blockingTimesMicroseconds.size(); i++)
    {
        blockingTimeSum += blockingTimesMicroseconds[i];
    }
    double meanBlockingTime = blockingTimeSum / blockingTimesMicroseconds.size();
    double meanProcessingTime = (1e6 / expectedFrameRate) - meanBlockingTime;
    ASSERT_LT(meanProcessingTime, maxPerFrameProcessingTimeAllowedMicroseconds)
        << "Mean processing time: " << meanProcessingTime << " us; "
        << "max allowed processing time: "
        << maxPerFrameProcessingTimeAllowedMicroseconds << " us";
}