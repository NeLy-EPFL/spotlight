#pragma once

#include <iostream>
#include <string>

#include <spdlog/spdlog.h>

struct CLIOptions {
    std::string profile_dir = "";
    std::string arena_dir = "";
    // run-spotlight and run-spotlight-headless only: run without the muscle
    // cameras
    bool no_muscle = false;
    // run-spotlight-headless only: recording parameters file (in the format of
    // a recording's metadata/experiment_parameters.yaml), save directory,
    // length of an open recording, and streaming time before recording
    std::string params_file = "";
    std::string save_dir = "";
    double duration_s = 0;
    double stream_s = 0;
    spdlog::level::level_enum log_level = spdlog::level::info;
};

CLIOptions parse_cli(int argc, char **argv);
