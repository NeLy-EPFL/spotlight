// Unit tests for MuscleCameraROI (recorder/src/peripherals/muscle_camera.cc):
// the derived offsets/dimensions, bounds checking against the sensor, the
// centre helper, and the YAML persistence round-trip of both cameras' ROIs
// through get_muscle_camera_rois.

#include "recorder/peripherals/muscle_camera.h"

#include <stdexcept>
#include <tuple>

#include <gtest/gtest.h>

#include "test_helpers.h"

namespace {

TEST(MuscleCameraROI, ComputesOffsetsAndDimensions) {
    // ROI is 1-based and inclusive: x in [11, 110], y in [21, 220].
    MuscleCameraROI roi(/*x0=*/11, /*x1=*/110, /*y0=*/21, /*y1=*/220);
    EXPECT_EQ(roi.x_offset, 10);      // x0 - 1
    EXPECT_EQ(roi.y_offset, 20);      // y0 - 1
    EXPECT_EQ(roi.image_width, 100);  // x1 - x0 + 1
    EXPECT_EQ(roi.image_height, 200); // y1 - y0 + 1
}

TEST(MuscleCameraROI, CenterIsMidpoint) {
    auto [cx, cy] = MuscleCameraROI(10, 20, 100, 200).get_center_xy();
    EXPECT_EQ(cx, 15);  // (10 + 20) / 2
    EXPECT_EQ(cy, 150); // (100 + 200) / 2
}

TEST(MuscleCameraROI, BoundsChecking) {
    // Exactly fills a 640x480 sensor (bounds are inclusive).
    EXPECT_TRUE(MuscleCameraROI(1, 640, 1, 480).is_within_bound(640, 480));
    // One past the right / bottom edge.
    EXPECT_FALSE(MuscleCameraROI(1, 641, 1, 480).is_within_bound(640, 480));
    EXPECT_FALSE(MuscleCameraROI(1, 640, 1, 481).is_within_bound(640, 480));
    // x0/y0 must be strictly positive (1-based).
    EXPECT_FALSE(MuscleCameraROI(0, 100, 1, 100).is_within_bound(640, 480));
    // Empty or inverted ranges are rejected (x0 < x1 and y0 < y1 required).
    EXPECT_FALSE(MuscleCameraROI(100, 100, 1, 100).is_within_bound(640, 480));
    EXPECT_FALSE(MuscleCameraROI(200, 100, 1, 100).is_within_bound(640, 480));
}

TEST(MuscleCameraROI, FileRoundTrip) {
    TempDir dir;
    fs::path f = dir.file("muscle_camera_roi.yaml");

    MuscleCameraROIs rois{
        MuscleCameraROI(11, 110, 21, 220), MuscleCameraROI(31, 130, 1, 200)};
    ASSERT_EQ(rois.to_file(f), 0);

    MuscleCameraROIs loaded = get_muscle_camera_rois(f);
    EXPECT_EQ(loaded.calcium.x0, 11);
    EXPECT_EQ(loaded.calcium.x1, 110);
    EXPECT_EQ(loaded.calcium.y0, 21);
    EXPECT_EQ(loaded.calcium.y1, 220);
    EXPECT_EQ(loaded.calcium.image_width, 100);
    EXPECT_EQ(loaded.calcium.image_height, 200);
    EXPECT_EQ(loaded.fiducial.x0, 31);
    EXPECT_EQ(loaded.fiducial.y0, 1);
}

TEST(MuscleCameraROI, LoadingRejectsDifferentSizes) {
    TempDir dir;
    fs::path f = dir.file("muscle_camera_roi.yaml");
    MuscleCameraROIs rois{
        MuscleCameraROI(1, 100, 1, 100), MuscleCameraROI(1, 100, 1, 200)};
    ASSERT_EQ(rois.to_file(f), 0);
    EXPECT_THROW(get_muscle_camera_rois(f), std::runtime_error);
}

TEST(MuscleCameraROI, LoadingRejectsMissingKey) {
    TempDir dir;
    fs::path f = dir.file("incomplete.yaml");
    // The fiducial section and the calcium y1 are missing
    std::ofstream(f) << "calcium:\n  x0: 1\n  x1: 10\n  y0: 1\n";
    EXPECT_THROW(get_muscle_camera_rois(f), std::runtime_error);
}

} // namespace
