// Unit tests for the hardware-independent muscle-camera timing logic
// (recorder/src/peripherals/muscle_camera.cc): MuscleTriggerTiming, which
// derives the continuous-mode (auto-sequence) exposure parameters from the
// recording configuration. The MuscleCamera class itself drives the PCO server
// over shared memory and is left uncovered here.

#include "recorder/peripherals/muscle_camera.h"

#include <gtest/gtest.h>

namespace {

// Reference configuration with round numbers so the derived values are easy to
// check by hand:
//   muscleFPS    = 100 / 2            = 50 Hz
//   interval     = 1e6 / 50           = 20000 us
//   rollingTime  = 200 * 20.0         = 4000 us
//   nominal      = 20000 - 1000       = 19000 us
//   commonTime   = 19000 - 4000       = 15000 us
//   bufferTime   = 15000 - 5000       = 10000 us
constexpr int kBehaviorFPS = 100;
constexpr int kSyncRatio = 2;
constexpr int kLightOnUs = 5000;
constexpr int kImageHeight = 200;
constexpr double kLineScanTimeUs = 20.0;
constexpr int kReadoutUs = 1000;

TEST(MuscleTriggerTiming, DerivesContinuousModeParameters) {
    MuscleTriggerTiming timing(kBehaviorFPS, kSyncRatio, kLightOnUs);
    ASSERT_TRUE(
        timing.computeParameters(kImageHeight, kLineScanTimeUs, kReadoutUs));

    EXPECT_EQ(timing.getNominalExposureUs(), 19000);
    EXPECT_EQ(timing.getCommonTimeUs(), 15000);
    EXPECT_EQ(timing.getBufferTimeUs(), 10000);
}

// The defining relationships documented on MuscleTriggerTiming:
//   nominalExposure = rollingTime + commonTime
//   commonTime      = lightOn + bufferTime
// Pin them so a future change to the derivation can't silently break the
// invariants the firmware and metadata rely on.
TEST(MuscleTriggerTiming, SatisfiesTimingInvariants) {
    MuscleTriggerTiming timing(kBehaviorFPS, kSyncRatio, kLightOnUs);
    ASSERT_TRUE(
        timing.computeParameters(kImageHeight, kLineScanTimeUs, kReadoutUs));

    const int rollingTimeUs = static_cast<int>(kImageHeight * kLineScanTimeUs);
    EXPECT_EQ(
        timing.getNominalExposureUs(),
        rollingTimeUs + timing.getCommonTimeUs());
    EXPECT_EQ(
        timing.getCommonTimeUs(), kLightOnUs + timing.getBufferTimeUs());
}

// A buffer time of exactly zero (the light-on window exactly fills the common
// time) is still a valid configuration: validity requires bufferTime >= 0.
TEST(MuscleTriggerTiming, ZeroBufferIsValid) {
    // syncRatio 1 => interval 10000; nominal 9000; commonTime 5000. A 5000 us
    // light-on time leaves exactly zero buffer.
    MuscleTriggerTiming timing(kBehaviorFPS, /*syncRatio=*/1, /*lightOn=*/5000);
    ASSERT_TRUE(
        timing.computeParameters(kImageHeight, kLineScanTimeUs, kReadoutUs));

    EXPECT_EQ(timing.getBufferTimeUs(), 0);
}

// When the muscle interval is too short to fit the light-on window inside the
// common time, the buffer time would be negative and the configuration is
// rejected.
TEST(MuscleTriggerTiming, RejectsLightOnLongerThanCommonTime) {
    // Same as ZeroBufferIsValid but with a light-on time 1 us too long.
    MuscleTriggerTiming timing(kBehaviorFPS, /*syncRatio=*/1, /*lightOn=*/5001);
    EXPECT_FALSE(
        timing.computeParameters(kImageHeight, kLineScanTimeUs, kReadoutUs));
}

} // namespace
