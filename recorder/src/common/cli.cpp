#include "cli.hpp"

void printHelp(const char* programName) {
    std::cout << "Usage: " << programName << " [OPTIONS]\n"
              << "Options:\n"
              << "  -h, --help                 Display this help message\n"
              << "  -p, --profile-dir PATH     Path to profile directory (required)\n"
              << "  -a, --arena PATH           Path to arena directory (containing metadata.yaml and model/)\n"
              << "  -v, --verbose              Enable verbose output (debug level)\n"
              << "  --verbosity LEVEL          Set verbosity level (trace, debug, info, warn, error, critical, off)\n"
              << std::endl;
}

spdlog::level::level_enum parseLogLevel(const std::string& level) {
    if (level == "trace") return spdlog::level::trace;
    if (level == "debug") return spdlog::level::debug;
    if (level == "info") return spdlog::level::info;
    if (level == "warn") return spdlog::level::warn;
    if (level == "error") return spdlog::level::err;
    if (level == "critical") return spdlog::level::critical;
    if (level == "off") return spdlog::level::off;
    
    spdlog::error("Unknown log level: {}. Using 'info'.", level);
    return spdlog::level::info;
}

CLIOptions parseCLI(int argc, char** argv) {
    CLIOptions options;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            std::exit(0);
        } else if (arg == "-v" || arg == "--verbose") {
            options.logLevel = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            options.logLevel = parseLogLevel(argv[++i]);
        } else if ((arg == "-p" || arg == "--profile-dir") && i + 1 < argc) {
            options.profileDir = argv[++i];
        } else if ((arg == "-a" || arg == "--arena") && i + 1 < argc) {
            options.arenaDir = argv[++i];
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printHelp(argv[0]);
            std::exit(1);
        }
    }

    if (options.profileDir.empty()) {
        std::cerr << "Error: -p/--profile-dir is required.\n";
        printHelp(argv[0]);
        std::exit(1);
    }

    return options;
}