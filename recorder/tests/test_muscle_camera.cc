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
//   muscle_fps    = 100 / 2           = 50 Hz
//   interval      = 1e6 / 50          = 20000 us
//   rolling_time  = 200 * 20.0        = 4000 us
//   nominal       = 20000 - 1000      = 19000 us
//   common_time   = 19000 - 4000      = 15000 us
//   buffer_time   = 15000 - 5000      = 10000 us
constexpr int behavior_fps = 100;
constexpr int sync_ratio = 2;
constexpr int light_on_us = 5000;
constexpr int image_height = 200;
constexpr double line_scan_time_us = 20.0;
constexpr int readout_us = 1000;

TEST(MuscleTriggerTiming, DerivesContinuousModeParameters) {
    MuscleTriggerTiming timing(behavior_fps, sync_ratio, light_on_us);
    ASSERT_TRUE(
        timing.compute_parameters(image_height, line_scan_time_us, readout_us));

    EXPECT_EQ(timing.get_nominal_exposure_us(), 19000);
    EXPECT_EQ(timing.get_common_time_us(), 15000);
    EXPECT_EQ(timing.get_buffer_time_us(), 10000);
}

// The defining relationships documented on MuscleTriggerTiming:
//   nominal_exposure = rolling_time + common_time
//   common_time      = light_on + buffer_time
// Pin them so a future change to the derivation can't silently break the
// invariants the firmware and metadata rely on.
TEST(MuscleTriggerTiming, SatisfiesTimingInvariants) {
    MuscleTriggerTiming timing(behavior_fps, sync_ratio, light_on_us);
    ASSERT_TRUE(
        timing.compute_parameters(image_height, line_scan_time_us, readout_us));

    const int rolling_time_us =
        static_cast<int>(image_height * line_scan_time_us);
    EXPECT_EQ(
        timing.get_nominal_exposure_us(),
        rolling_time_us + timing.get_common_time_us());
    EXPECT_EQ(
        timing.get_common_time_us(), light_on_us + timing.get_buffer_time_us());
}

// A buffer time of exactly zero (the light-on window exactly fills the common
// time) is still a valid configuration: validity requires buffer_time >= 0.
TEST(MuscleTriggerTiming, ZeroBufferIsValid) {
    // sync_ratio 1 => interval 10000; nominal 9000; common_time 5000. A 5000 us
    // light-on time leaves exactly zero buffer.
    MuscleTriggerTiming timing(
        behavior_fps, /*sync_ratio=*/1, /*light_on=*/5000);
    ASSERT_TRUE(
        timing.compute_parameters(image_height, line_scan_time_us, readout_us));

    EXPECT_EQ(timing.get_buffer_time_us(), 0);
}

// When the muscle interval is too short to fit the light-on window inside the
// common time, the buffer time would be negative and the configuration is
// rejected.
TEST(MuscleTriggerTiming, RejectsLightOnLongerThanCommonTime) {
    // Same as ZeroBufferIsValid but with a light-on time 1 us too long.
    MuscleTriggerTiming timing(
        behavior_fps, /*sync_ratio=*/1, /*light_on=*/5001);
    EXPECT_FALSE(
        timing.compute_parameters(image_height, line_scan_time_us, readout_us));
}

} // namespace
