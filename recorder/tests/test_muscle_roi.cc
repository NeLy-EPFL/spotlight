// Unit tests for MuscleCameraROI (recorder/src/common/muscle_recording.cc): the
// derived offsets/dimensions, bounds checking against the sensor, the centre
// helper, and the YAML persistence round-trip through getMuscleCameraROI.

#include "recorder/common/muscle_recording.h"

#include <stdexcept>
#include <tuple>

#include <gtest/gtest.h>

#include "test_helpers.h"

namespace {

TEST(MuscleCameraROI, ComputesOffsetsAndDimensions) {
    // ROI is 1-based and inclusive: x in [11, 110], y in [21, 220].
    MuscleCameraROI roi(/*x0=*/11, /*x1=*/110, /*y0=*/21, /*y1=*/220);
    EXPECT_EQ(roi.xOffset, 10);      // x0 - 1
    EXPECT_EQ(roi.yOffset, 20);      // y0 - 1
    EXPECT_EQ(roi.imageWidth, 100);  // x1 - x0 + 1
    EXPECT_EQ(roi.imageHeight, 200); // y1 - y0 + 1
}

TEST(MuscleCameraROI, CenterIsMidpoint) {
    auto [cx, cy] = MuscleCameraROI(10, 20, 100, 200).getCenterXY();
    EXPECT_EQ(cx, 15);  // (10 + 20) / 2
    EXPECT_EQ(cy, 150); // (100 + 200) / 2
}

TEST(MuscleCameraROI, BoundsChecking) {
    // Exactly fills a 640x480 sensor (bounds are inclusive).
    EXPECT_TRUE(MuscleCameraROI(1, 640, 1, 480).isWithinBound(640, 480));
    // One past the right / bottom edge.
    EXPECT_FALSE(MuscleCameraROI(1, 641, 1, 480).isWithinBound(640, 480));
    EXPECT_FALSE(MuscleCameraROI(1, 640, 1, 481).isWithinBound(640, 480));
    // x0/y0 must be strictly positive (1-based).
    EXPECT_FALSE(MuscleCameraROI(0, 100, 1, 100).isWithinBound(640, 480));
    // Empty or inverted ranges are rejected (x0 < x1 and y0 < y1 required).
    EXPECT_FALSE(MuscleCameraROI(100, 100, 1, 100).isWithinBound(640, 480));
    EXPECT_FALSE(MuscleCameraROI(200, 100, 1, 100).isWithinBound(640, 480));
}

TEST(MuscleCameraROI, FileRoundTrip) {
    TempDir dir;
    fs::path f = dir.file("muscle_camera_roi.yaml");

    MuscleCameraROI roi(11, 110, 21, 220);
    ASSERT_EQ(roi.toFile(f), 0);

    MuscleCameraROI loaded = getMuscleCameraROI(f);
    EXPECT_EQ(loaded.x0, 11);
    EXPECT_EQ(loaded.x1, 110);
    EXPECT_EQ(loaded.y0, 21);
    EXPECT_EQ(loaded.y1, 220);
    EXPECT_EQ(loaded.imageWidth, 100);
    EXPECT_EQ(loaded.imageHeight, 200);
}

TEST(MuscleCameraROI, LoadingRejectsMissingKey) {
    TempDir dir;
    fs::path f = dir.file("incomplete.yaml");
    std::ofstream(f) << "x0: 1\nx1: 10\ny0: 1\n"; // y1 and the dims are missing
    EXPECT_THROW(getMuscleCameraROI(f), std::runtime_error);
}

} // namespace
