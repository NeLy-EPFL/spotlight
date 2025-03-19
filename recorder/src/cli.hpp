#include <string>
#include <iostream>

#include <spdlog/spdlog.h>

struct CLIOptions {
    std::string configPath;
    spdlog::level::level_enum logLevel;
};

CLIOptions parseCLI(int argc, char** argv);