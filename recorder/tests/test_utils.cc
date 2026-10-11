// Unit tests for the recorder's hardware-independent utilities
// (recorder/src/common/utils.cc): image reorientation, pseudo-BGR frame
// packing, per-frame metadata, unit conversions, path expansion and the small
// file/IO helpers. Functions that enumerate serial ports talk to hardware and
// are intentionally left uncovered.

#include "recorder/common/utils.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "test_helpers.h"

namespace {

/* -------------------------------------------------------------------------- */
/* Image reorientation                                                        */
/* -------------------------------------------------------------------------- */

// The four corner values of a single-channel 8-bit image.
struct Corners {
    int tl, tr, bl, br;
};

Corners corners_of(const cv::Mat &m) {
    return {
        m.at<uchar>(0, 0),
        m.at<uchar>(0, m.cols - 1),
        m.at<uchar>(m.rows - 1, 0),
        m.at<uchar>(m.rows - 1, m.cols - 1)};
}

// A 2x3 image (2 rows, 3 cols) with a distinct non-zero marker in each corner,
// so a reorientation can be pinned by where each marker ends up.
cv::Mat marked_image() {
    cv::Mat m = cv::Mat::zeros(2, 3, CV_8UC1);
    m.at<uchar>(0, 0) = 11; // top-left
    m.at<uchar>(0, 2) = 22; // top-right
    m.at<uchar>(1, 0) = 33; // bottom-left
    m.at<uchar>(1, 2) = 44; // bottom-right
    return m;
}

TEST(Reorient, MuscleRotates90CounterClockwise) {
    cv::Mat dst;
    reorient_muscle_image(marked_image(), dst);

    // A 90-degree rotation swaps the dimensions.
    EXPECT_EQ(dst.rows, 3);
    EXPECT_EQ(dst.cols, 2);

    // Counter-clockwise: the source top-right corner becomes the new top-left,
    // and the rest follow around the frame.
    Corners c = corners_of(dst);
    EXPECT_EQ(c.tl, 22); // from source top-right
    EXPECT_EQ(c.tr, 44); // from source bottom-right
    EXPECT_EQ(c.bl, 11); // from source top-left
    EXPECT_EQ(c.br, 33); // from source bottom-left
}

TEST(Reorient, BehaviorRotatesThenMirrorsHorizontally) {
    cv::Mat dst;
    reorient_behavior_image(marked_image(), dst);

    EXPECT_EQ(dst.rows, 3);
    EXPECT_EQ(dst.cols, 2);

    // 90-degree CCW rotation followed by a horizontal flip leaves the
    // top-right and bottom-left corners in place and swaps the other diagonal.
    Corners c = corners_of(dst);
    EXPECT_EQ(c.tl, 44); // from source bottom-right
    EXPECT_EQ(c.tr, 22); // from source top-right
    EXPECT_EQ(c.bl, 33); // from source bottom-left
    EXPECT_EQ(c.br, 11); // from source top-left
}

/* -------------------------------------------------------------------------- */
/* Pseudo-BGR packing                                                         */
/* -------------------------------------------------------------------------- */

GroupOfThreeFrames make_uniform_group(int v0, int v1, int v2, int num_valid) {
    GroupOfThreeFrames group;
    group.frame0.image = cv::Mat(4, 5, CV_8UC1, cv::Scalar(v0));
    group.frame1.image = cv::Mat(4, 5, CV_8UC1, cv::Scalar(v1));
    group.frame2.image = cv::Mat(4, 5, CV_8UC1, cv::Scalar(v2));
    group.num_valid_frames = num_valid;
    return group;
}

TEST(PseudoBGR, PacksThreeFramesIntoChannelsInOrder) {
    cv::Mat bgr = make_pseudo_bgr_image_from_three_frames(
        make_uniform_group(10, 20, 30, 3));

    ASSERT_EQ(bgr.channels(), 3);
    // Reorientation swaps the dimensions but preserves pixel values.
    EXPECT_EQ(bgr.rows, 5);
    EXPECT_EQ(bgr.cols, 4);

    std::vector<cv::Mat> ch;
    cv::split(bgr, ch);
    EXPECT_EQ(cv::countNonZero(ch[0] != 10), 0); // frame0 -> channel 0
    EXPECT_EQ(cv::countNonZero(ch[1] != 20), 0); // frame1 -> channel 1
    EXPECT_EQ(cv::countNonZero(ch[2] != 30), 0); // frame2 -> channel 2
}

TEST(PseudoBGR, PartialGroupBlacksOutMissingChannels) {
    // Only one valid frame: the other two channels must be black even though
    // frame1/frame2 carry data (a partial final group fills them with black).
    std::vector<cv::Mat> ch;
    cv::split(
        make_pseudo_bgr_image_from_three_frames(
            make_uniform_group(10, 20, 30, 1)),
        ch);

    EXPECT_EQ(cv::countNonZero(ch[0] != 10), 0);
    EXPECT_EQ(cv::countNonZero(ch[1]), 0);
    EXPECT_EQ(cv::countNonZero(ch[2]), 0);
}

TEST(PseudoBGR, TwoFrameGroupKeepsOnlyThirdChannelBlack) {
    std::vector<cv::Mat> ch;
    cv::split(
        make_pseudo_bgr_image_from_three_frames(
            make_uniform_group(10, 20, 30, 2)),
        ch);

    EXPECT_EQ(cv::countNonZero(ch[0] != 10), 0);
    EXPECT_EQ(cv::countNonZero(ch[1] != 20), 0);
    EXPECT_EQ(cv::countNonZero(ch[2]), 0);
}

/* -------------------------------------------------------------------------- */
/* Per-frame metadata                                                         */
/* -------------------------------------------------------------------------- */

FrameData make_frame(unsigned int id, uint64_t acquired, uint64_t received) {
    FrameData f;
    f.frame_id = id;
    f.acquisition_time = acquired;
    f.received_time = received;
    return f;
}

std::vector<std::string> split_lines(const std::string &s) {
    std::vector<std::string> lines;
    std::string line;
    std::istringstream stream(s);
    while (std::getline(stream, line)) {
        lines.push_back(line);
    }
    return lines;
}

TEST(Metadata, FullGroupLogsHeaderAndThreeRows) {
    GroupOfThreeFrames group;
    group.frame0 = make_frame(0, 100, 101);
    group.frame1 = make_frame(1, 200, 202);
    group.frame2 = make_frame(2, 300, 303);
    group.num_valid_frames = 3;

    std::vector<std::string> lines =
        split_lines(make_metadata_string_from_three_frames(group));
    ASSERT_EQ(lines.size(), 4u); // header + 3 rows
    EXPECT_EQ(lines[0], "frame_id,acquired_time_us,received_time_us");
    EXPECT_EQ(lines[1], "0,100,101");
    EXPECT_EQ(lines[2], "1,200,202");
    EXPECT_EQ(lines[3], "2,300,303");
}

TEST(Metadata, PartialGroupOnlyLogsValidFrames) {
    GroupOfThreeFrames group;
    group.frame0 = make_frame(7, 700, 701);
    group.frame1 = make_frame(8, 800, 802); // not logged
    group.frame2 = make_frame(9, 900, 903); // not logged
    group.num_valid_frames = 1;

    std::vector<std::string> lines =
        split_lines(make_metadata_string_from_three_frames(group));
    ASSERT_EQ(lines.size(), 2u); // header + 1 row
    EXPECT_EQ(lines[1], "7,700,701");
}

/* -------------------------------------------------------------------------- */
/* Numeric helpers                                                            */
/* -------------------------------------------------------------------------- */

TEST(PreviewWidth, ScalesHeightByStageAspectRatio) {
    EXPECT_EQ(calculate_behavior_camera_preview_width(100, 3, 2), 150);
    EXPECT_EQ(calculate_behavior_camera_preview_width(100, 2, 2), 100);
    // The float ratio is truncated toward zero when cast back to int.
    EXPECT_EQ(calculate_behavior_camera_preview_width(100, 2, 3), 66);
}

TEST(Convert16To8, MapsWindowOntoEightBitRangeWithClamping) {
    cv::Mat src(1, 3, CV_16UC1);
    src.at<uint16_t>(0, 0) = 100;
    src.at<uint16_t>(0, 1) = 500;
    src.at<uint16_t>(0, 2) = 1000;

    cv::Mat dst;
    // Window [0, 1000] maps linearly: 500 lands at the midpoint, 1000 at white.
    convert16_bit_to8_bit(src, dst, /*vmin=*/0, /*vmax=*/1000);
    ASSERT_EQ(dst.type(), CV_8UC1);
    EXPECT_EQ(dst.at<uchar>(0, 1), 128); // round(500 * 255 / 1000)
    EXPECT_EQ(dst.at<uchar>(0, 2), 255);

    // Window [200, 800]: pixels at/below vmin are black, at/above vmax white.
    convert16_bit_to8_bit(src, dst, /*vmin=*/200, /*vmax=*/800);
    EXPECT_EQ(dst.at<uchar>(0, 0), 0);   // 100 < vmin -> black
    EXPECT_EQ(dst.at<uchar>(0, 1), 128); // round((500 - 200) * 255 / 600)
    EXPECT_EQ(dst.at<uchar>(0, 2), 255); // 1000 > vmax -> white
}

TEST(CurrentTime, IsPositiveAndNonDecreasing) {
    uint64_t t0 = get_current_time_microseconds();
    uint64_t t1 = get_current_time_microseconds();
    EXPECT_GT(t0, 0u);
    EXPECT_GE(t1, t0);
}

/* -------------------------------------------------------------------------- */
/* Path expansion                                                             */
/* -------------------------------------------------------------------------- */

TEST(ExpandPath, ExpandsLeadingTilde) {
    EnvGuard home("HOME", "/home/tester");
    EXPECT_EQ(expand_path("~/data/run1"), "/home/tester/data/run1");
}

TEST(ExpandPath, LeavesOtherPathsUntouched) {
    EnvGuard home("HOME", "/home/tester");
    EXPECT_EQ(expand_path("/abs/path"), "/abs/path");
    EXPECT_EQ(expand_path("relative/path"), "relative/path");
    EXPECT_EQ(expand_path("~"), "~");                   // no trailing slash
    EXPECT_EQ(expand_path("~user/path"), "~user/path"); // not the "~/" form
}

/* -------------------------------------------------------------------------- */
/* File / IO helpers                                                          */
/* -------------------------------------------------------------------------- */

// NOTE: the experiment-parameters YAML writer used to be a free function
// (write_experiment_parameters) and was unit-tested here. It now lives as a
// private MainGUIWindow method that reads the recording parameters directly off
// the GUI widgets, so it is no longer reachable from these hardware-independent
// tests. The two tests that covered it were removed with that refactor.

TEST(PrepareOutputFolder, CreatesDirectoryAndReturnsAbsolutePath) {
    TempDir dir;
    fs::path result = prepare_output_folder(dir.file("nested/output"), false);
    EXPECT_TRUE(fs::exists(result));
    EXPECT_TRUE(result.is_absolute());
}

TEST(PrepareOutputFolder, ClearsExistingContentWhenAsked) {
    TempDir dir;
    fs::path target = dir.file("output");
    fs::create_directories(target);
    std::ofstream(target / "stale.txt") << "old";
    ASSERT_TRUE(fs::exists(target / "stale.txt"));

    prepare_output_folder(target, /*clear_folder=*/true);
    EXPECT_FALSE(fs::exists(target / "stale.txt"));
    EXPECT_TRUE(fs::exists(target));
}

TEST(PrepareOutputFolder, KeepsContentWhenNotClearing) {
    TempDir dir;
    fs::path target = dir.file("output");
    fs::create_directories(target);
    std::ofstream(target / "keep.txt") << "data";

    prepare_output_folder(target, /*clear_folder=*/false);
    EXPECT_TRUE(fs::exists(target / "keep.txt"));
}

TEST(SaveDirectory, InitializeCreatesRecordingSubdirectories) {
    TempDir dir;
    fs::path root = dir.file("recording");
    fs::create_directories(root);

    SaveDirectory save_dir(root.string());
    EXPECT_EQ(save_dir.get_directory(), root);

    save_dir.initialize();
    EXPECT_TRUE(fs::is_directory(root / "behavior_images"));
    EXPECT_TRUE(fs::is_directory(root / "muscle_images"));
    EXPECT_TRUE(fs::is_directory(root / "stage_position"));
    EXPECT_TRUE(fs::is_directory(root / "metadata"));
}

TEST(SaveDirectory, ExpandsLeadingTildeInDirectory) {
    TempDir dir;
    EnvGuard home("HOME", dir.path().string());
    SaveDirectory save_dir("~/myrun");
    EXPECT_EQ(save_dir.get_directory(), dir.path() / "myrun");
}

TEST(LatestFrame, RoundTripsLatestFrameData) {
    LatestFrame holder;
    holder.set_latest_frame_data(make_frame(42, 123456, 123999));

    FrameData out = holder.get_latest_frame_data();
    EXPECT_EQ(out.frame_id, 42u);
    EXPECT_EQ(out.acquisition_time, 123456u);
    EXPECT_EQ(out.received_time, 123999u);
}

/* -------------------------------------------------------------------------- */
/* Position-dependent tracking acceleration                                   */
/* -------------------------------------------------------------------------- */

// 200x100 image -> center at (100, 50); r_max = min(100, 200)/2 - 10 = 40.
constexpr int kCols = 200;
constexpr int kRows = 100;
constexpr double kAccelMin = 20.0;
constexpr double kAccelMax = 250.0;
constexpr double kMargin = 10.0;

double accel_at(double col, double row) {
    return compute_tracking_acceleration(
        col, row, kCols, kRows, kAccelMin, kAccelMax, kMargin);
}

TEST(ComputeTrackingAcceleration, ReturnsAccelMinAtCenter) {
    EXPECT_DOUBLE_EQ(accel_at(kCols / 2.0, kRows / 2.0), kAccelMin);
}

TEST(ComputeTrackingAcceleration, ReturnsAccelMaxAtEdgeRadius) {
    // 40 px straight down from center reaches exactly r_max.
    EXPECT_DOUBLE_EQ(accel_at(kCols / 2.0, kRows / 2.0 + 40.0), kAccelMax);
}

TEST(ComputeTrackingAcceleration, ClampsToAccelMaxBeyondEdgeRadius) {
    EXPECT_DOUBLE_EQ(accel_at(kCols / 2.0, kRows / 2.0 + 80.0), kAccelMax);
}

TEST(ComputeTrackingAcceleration, InterpolatesLinearlyAtMidpoint) {
    // Half of r_max -> halfway between accel_min and accel_max.
    double midpoint = kAccelMin + 0.5 * (kAccelMax - kAccelMin);
    EXPECT_DOUBLE_EQ(accel_at(kCols / 2.0, kRows / 2.0 + 20.0), midpoint);
}

TEST(ComputeTrackingAcceleration, ClampsToAccelMaxWhenMarginExceedsHalfDim) {
    // Degenerate r_max <= 0: every position should map to accel_max.
    EXPECT_DOUBLE_EQ(
        compute_tracking_acceleration(
            kCols / 2.0, kRows / 2.0, kCols, kRows, kAccelMin, kAccelMax,
            /*max_accel_margin_px=*/100.0),
        kAccelMax);
}

} // namespace
