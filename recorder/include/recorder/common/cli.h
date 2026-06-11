#pragma once

#include <iostream>
#include <string>

#include <spdlog/spdlog.h>

struct CLIOptions {
    std::string profile_dir = "";
    std::string arena_dir = "";
    spdlog::level::level_enum log_level = spdlog::level::info;
};

CLIOptions parse_cli(int argc, char **argv);
