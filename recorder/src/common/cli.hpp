#include <string>
#include <iostream>

#include <spdlog/spdlog.h>

struct CLIOptions {
    std::string profileDir = "~/Spotlight/default/";
    spdlog::level::level_enum logLevel = spdlog::level::info;
};

CLIOptions parseCLI(int argc, char** argv);