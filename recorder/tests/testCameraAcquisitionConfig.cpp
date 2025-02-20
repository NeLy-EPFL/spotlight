#include <gtest/gtest.h>

#include "../src/dataTypes.hpp"
#include "../src/utils.hpp"

TEST(CameraAcquisitionConfigTest, InitFromValues)
{
    CameraAcquisitionConfig config(CameraAcquisitionMode::RECORD, 10, 1234);
    EXPECT_EQ(config.mode, CameraAcquisitionMode::RECORD);
    EXPECT_EQ(config.fps, 10);
    EXPECT_EQ(config.exposureTimeMicroseconds, 1234);
}

TEST(CameraAcquisitionConfigTest, InitFromCommandString)
{
    CameraAcquisitionConfig config("0 20 1234");
    EXPECT_EQ(config.mode, CameraAcquisitionMode::STREAM);
    EXPECT_EQ(config.fps, 20);
    EXPECT_EQ(config.exposureTimeMicroseconds, 1234);
}

TEST(CameraAcquisitionConfigTest, ToCommandString)
{
    CameraAcquisitionConfig config(CameraAcquisitionMode::RECORD, 30, 1234);
    EXPECT_EQ(config.toCommandString(), "1 30 1234");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}