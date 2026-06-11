#include "recorder/common/cli.h"

void print_help(const char *program_name) {
    // clang-format off
    std::cout << "Usage: " << program_name << " [OPTIONS]\n"
              << "Options:\n"
              << "  -h, --help                 Display this help message\n"
              << "  -p, --profile-dir PATH     Path to profile directory (required)\n"
              << "  -a, --arena PATH           Path to arena directory (containing metadata.yaml and model/)\n"
              << "  -v, --verbose              Enable verbose output (debug level)\n"
              << "  --verbosity LEVEL          Set verbosity level (trace, debug, info, warn, error, critical, off)\n"
              << std::endl;
    // clang-format on
}

spdlog::level::level_enum parse_log_level(const std::string &level) {
    if (level == "trace")
        return spdlog::level::trace;
    if (level == "debug")
        return spdlog::level::debug;
    if (level == "info")
        return spdlog::level::info;
    if (level == "warn")
        return spdlog::level::warn;
    if (level == "error")
        return spdlog::level::err;
    if (level == "critical")
        return spdlog::level::critical;
    if (level == "off")
        return spdlog::level::off;

    spdlog::error("Unknown log level: {}. Using 'info'.", level);
    return spdlog::level::info;
}

CLIOptions parse_cli(int argc, char **argv) {
    CLIOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            std::exit(0);
        } else if (arg == "-v" || arg == "--verbose") {
            options.log_level = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            options.log_level = parse_log_level(argv[++i]);
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc) {
            options.profile_dir = argv[++i];
        } else if ((arg == "-a" || arg == "--arena") && i + 1 < argc) {
            options.arena_dir = argv[++i];
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_help(argv[0]);
            std::exit(1);
        }
    }

    if (options.profile_dir.empty()) {
        std::cerr << "Error: -p/--profile-dir is required.\n";
        print_help(argv[0]);
        std::exit(1);
    }

    return options;
}