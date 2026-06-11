// Shared helpers for the hardware-dependent recorder tests.
//
// The macro REQUIRE_SPOTLIGHT_PROFILE_DIR must be invoked directly in a TEST
// or TEST_F body (not inside a helper function), because GTEST_SKIP() expands
// to a return statement that only returns from the immediately enclosing
// function.
#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "recorder/common/recorder_config.h"

// Reads SPOTLIGHT_PROFILE_DIR from the environment and binds it to `varname`.
// Issues GTEST_SKIP() and returns from the calling TEST body if the variable
// is unset or empty.
#define REQUIRE_SPOTLIGHT_PROFILE_DIR(varname)                                 \
    const char *_##varname##_env = std::getenv("SPOTLIGHT_PROFILE_DIR");       \
    if (!_##varname##_env || _##varname##_env[0] == '\0') {                    \
        GTEST_SKIP() << "SPOTLIGHT_PROFILE_DIR not set; "                      \
                        "skipping hardware test";                              \
    }                                                                          \
    std::string varname(_##varname##_env)

// Load recorder_config.yaml from a profile directory.
inline RecorderConfig load_recorder_config(const std::string &profile_dir) {
    namespace fs = std::filesystem;
    return RecorderConfig(
        (fs::path(profile_dir) / "recorder_config.yaml").string());
}
