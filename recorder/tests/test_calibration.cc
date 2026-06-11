// Unit tests for the calibration model (recorder/src/common/calibration.cc):
// the linear 2x2-to-2 mapper and the CalibrationParams loader, with particular
// attention to the analytic inversion of the stage block (physical+pixel ->
// stage) that the tracking loop relies on.

#include "recorder/common/calibration.h"

#include <stdexcept>
#include <tuple>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "test_helpers.h"

namespace {

// A well-conditioned calibration fixture. The forward stage+pixel -> physical
// map is, with col = pixel x and row = pixel y:
//   physical_x = stage_x + 0.5 * col + 100
//   physical_y = stage_y + 0.5 * row + 200
// and stage+physical -> pixel is its exact inverse for the pixel coordinate.
void write_calib_file(const fs::path &path) {
    std::ofstream(path) << R"(stage_and_pixel_to_physical:
  physical_pos_x:
    stage_pos_x: 1.0
    stage_pos_y: 0.0
    pixel_pos_x: 0.5
    pixel_pos_y: 0.0
    bias: 100.0
  physical_pos_y:
    stage_pos_x: 0.0
    stage_pos_y: 1.0
    pixel_pos_x: 0.0
    pixel_pos_y: 0.5
    bias: 200.0
stage_and_physical_to_pixel:
  pixel_pos_x:
    stage_pos_x: -2.0
    stage_pos_y: 0.0
    physical_pos_x: 2.0
    physical_pos_y: 0.0
    bias: -200.0
  pixel_pos_y:
    stage_pos_x: 0.0
    stage_pos_y: -2.0
    physical_pos_x: 0.0
    physical_pos_y: 2.0
    bias: -400.0
)";
}

// Both sections present, but the stage block of the forward map is singular
// (its two rows are identical), so it cannot be inverted.
void write_singular_calib_file(const fs::path &path) {
    std::ofstream(path) << R"(stage_and_pixel_to_physical:
  physical_pos_x:
    stage_pos_x: 1.0
    stage_pos_y: 1.0
    pixel_pos_x: 0.5
    pixel_pos_y: 0.0
    bias: 100.0
  physical_pos_y:
    stage_pos_x: 1.0
    stage_pos_y: 1.0
    pixel_pos_x: 0.0
    pixel_pos_y: 0.5
    bias: 200.0
stage_and_physical_to_pixel:
  pixel_pos_x:
    stage_pos_x: 0.0
    stage_pos_y: 0.0
    physical_pos_x: 1.0
    physical_pos_y: 0.0
    bias: 0.0
  pixel_pos_y:
    stage_pos_x: 0.0
    stage_pos_y: 0.0
    physical_pos_x: 0.0
    physical_pos_y: 1.0
    bias: 0.0
)";
}

/* -------------------------------------------------------------------------- */
/* LinearMapper2x2to2                                                         */
/* -------------------------------------------------------------------------- */

TEST(LinearMapper, DefaultMapsEverythingToZero) {
    LinearMapper2x2to2 m;
    auto [x, y] = m.map(1, 2, 3, 4);
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
}

TEST(LinearMapper, AppliesWeightsAndBias) {
    YAML::Node node;
    node["x"]["x1"] = 1.0;
    node["x"]["y1"] = 2.0;
    node["x"]["x2"] = 3.0;
    node["x"]["y2"] = 4.0;
    node["x"]["bias"] = 5.0;
    node["y"]["x1"] = 10.0;
    node["y"]["y1"] = 20.0;
    node["y"]["x2"] = 30.0;
    node["y"]["y2"] = 40.0;
    node["y"]["bias"] = 50.0;
    LinearMapper2x2to2 m(node);

    auto [x, y] = m.map(1.0, 1.0, 1.0, 1.0);
    EXPECT_DOUBLE_EQ(x, 1 + 2 + 3 + 4 + 5);      // 15
    EXPECT_DOUBLE_EQ(y, 10 + 20 + 30 + 40 + 50); // 150

    // Only the x1 weight contributes, but the bias is always added.
    std::tie(x, y) = m.map(2.0, 0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(x, 1 * 2.0 + 5);   // 7
    EXPECT_DOUBLE_EQ(y, 10 * 2.0 + 50); // 70
}

/* -------------------------------------------------------------------------- */
/* CalibrationParams                                                          */
/* -------------------------------------------------------------------------- */

TEST(CalibrationParams, UndefinedByDefaultAndThrowsOnUse) {
    CalibrationParams params;
    EXPECT_FALSE(params.is_defined);
    EXPECT_THROW(
        params.stage_pos_and_pixel_pos_to_physical_pos(0, 0, 0, 0),
        std::runtime_error);
}

TEST(CalibrationParams, MapsStageAndPixelToPhysical) {
    TempDir dir;
    fs::path f = dir.file("calib.yaml");
    write_calib_file(f);
    CalibrationParams params(f.string());
    ASSERT_TRUE(params.is_defined);

    auto [px, py] = params.stage_pos_and_pixel_pos_to_physical_pos(
        5, // stage_x
        7, // stage_y
        40, // row
        60); // col
    EXPECT_DOUBLE_EQ(px, 5 + 0.5 * 60 + 100); // 135
    EXPECT_DOUBLE_EQ(py, 7 + 0.5 * 40 + 200); // 227
}

TEST(CalibrationParams, PhysicalToStageInvertsStageToPhysical) {
    TempDir dir;
    fs::path f = dir.file("calib.yaml");
    write_calib_file(f);
    CalibrationParams params(f.string());

    const double sx = 5, sy = 7;
    const int row = 40, col = 60;
    auto [px, py] =
        params.stage_pos_and_pixel_pos_to_physical_pos(sx, sy, row, col);

    // Holding the pixel coordinate fixed, the analytically inverted map must
    // recover the original stage position.
    auto [rx, ry] =
        params.physical_pos_and_pixel_pos_to_stage_pos(px, py, row, col);
    EXPECT_NEAR(rx, sx, 1e-9);
    EXPECT_NEAR(ry, sy, 1e-9);
}

TEST(CalibrationParams, StageAndPhysicalToPixelRoundTrips) {
    TempDir dir;
    fs::path f = dir.file("calib.yaml");
    write_calib_file(f);
    CalibrationParams params(f.string());

    const double sx = 5, sy = 7;
    const int row = 40, col = 60;
    auto [px, py] =
        params.stage_pos_and_pixel_pos_to_physical_pos(sx, sy, row, col);

    auto [r, c] =
        params.stage_pos_and_physical_pos_to_pixel_pos(sx, sy, px, py);
    EXPECT_EQ(r, row); // result is returned as {row, col}
    EXPECT_EQ(c, col);
}

TEST(CalibrationParams, PixelResultIsRounded) {
    TempDir dir;
    fs::path f = dir.file("calib.yaml");
    write_calib_file(f);
    CalibrationParams params(f.string());

    // row = 2 * physical_y - 2 * stage_y - 400. With stage_y = 7:
    //   physical_y = 227.2 -> row = 40.4 -> rounds to 40
    //   physical_y = 227.3 -> row = 40.6 -> rounds to 41
    auto [r1, c1] =
        params.stage_pos_and_physical_pos_to_pixel_pos(5, 7, 135, 227.2);
    EXPECT_EQ(r1, 40);
    auto [r2, c2] =
        params.stage_pos_and_physical_pos_to_pixel_pos(5, 7, 135, 227.3);
    EXPECT_EQ(r2, 41);
}

TEST(CalibrationParams, RejectsSingularStageBlock) {
    TempDir dir;
    fs::path f = dir.file("singular.yaml");
    write_singular_calib_file(f);
    EXPECT_THROW(CalibrationParams params(f.string()), std::runtime_error);
}

TEST(CalibrationParams, RejectsMissingSection) {
    TempDir dir;
    fs::path f = dir.file("incomplete.yaml");
    // Only one of the two required sections is present.
    std::ofstream(f) << "stage_and_pixel_to_physical: {}\n";
    EXPECT_THROW(CalibrationParams params(f.string()), std::runtime_error);
}

TEST(CalibrationParams, SaveToFileRoundTripsThroughReload) {
    TempDir dir;
    fs::path f = dir.file("calib.yaml");
    write_calib_file(f);
    CalibrationParams params(f.string());

    fs::path out = dir.file("calib_out.yaml");
    params.save_to_file(out.string());

    CalibrationParams reloaded(out.string());
    auto [px, py] =
        reloaded.stage_pos_and_pixel_pos_to_physical_pos(5, 7, 40, 60);
    EXPECT_DOUBLE_EQ(px, 135);
    EXPECT_DOUBLE_EQ(py, 227);
}

} // namespace
