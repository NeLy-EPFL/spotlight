// Unit tests for RecorderConfig (recorder/src/common/recorder_config.cc): typed
// access to a YAML config tree, the "not loaded" / "missing key" error paths,
// and save/reload round-tripping.

#include "recorder/common/recorder_config.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "test_helpers.h"

namespace {

void writeConfig(const fs::path &path) {
    std::ofstream(path) << R"(camera:
  frame_rate_hz: 100
  exposure_time_ms: 2.5
  name: behavior
tracking:
  enabled: true
  update_frequency_hz: 30
)";
}

TEST(RecorderConfig, UndefinedByDefaultAndThrowsOnAccess) {
    RecorderConfig config;
    EXPECT_FALSE(config.isDefined);
    EXPECT_THROW(
        config.getParameter<int>("camera", "frame_rate_hz"),
        std::runtime_error);
}

TEST(RecorderConfig, ReadsTypedParameters) {
    TempDir dir;
    fs::path f = dir.file("recorder_config.yaml");
    writeConfig(f);
    RecorderConfig config(f.string());
    ASSERT_TRUE(config.isDefined);

    EXPECT_EQ(config.getParameter<int>("camera", "frame_rate_hz"), 100);
    EXPECT_FLOAT_EQ(
        config.getParameter<float>("camera", "exposure_time_ms"), 2.5f);
    EXPECT_EQ(config.getParameter<std::string>("camera", "name"), "behavior");
    EXPECT_TRUE(config.getParameter<bool>("tracking", "enabled"));
    EXPECT_EQ(config.getParameter<int>("tracking", "update_frequency_hz"), 30);
}

TEST(RecorderConfig, ThrowsOnMissingParameterOrSection) {
    TempDir dir;
    fs::path f = dir.file("recorder_config.yaml");
    writeConfig(f);
    RecorderConfig config(f.string());

    EXPECT_THROW(
        config.getParameter<int>("camera", "does_not_exist"),
        std::runtime_error);
    EXPECT_THROW(
        config.getParameter<int>("no_section", "frame_rate_hz"),
        std::runtime_error);
}

TEST(RecorderConfig, ThrowsWhenLoadingMissingFile) {
    EXPECT_THROW(
        RecorderConfig("/no/such/recorder_config.yaml"), std::runtime_error);
}

TEST(RecorderConfig, SaveToFileRoundTripsThroughReload) {
    TempDir dir;
    fs::path f = dir.file("recorder_config.yaml");
    writeConfig(f);
    RecorderConfig config(f.string());

    fs::path out = dir.file("out.yaml");
    config.saveToFile(out.string());

    RecorderConfig reloaded(out.string());
    EXPECT_EQ(reloaded.getParameter<int>("camera", "frame_rate_hz"), 100);
    EXPECT_EQ(reloaded.getParameter<std::string>("camera", "name"), "behavior");
}

} // namespace
