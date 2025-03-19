#include "cli.hpp"

void printHelp(const char* programName) {
    std::cout << "Usage: " << programName << " [OPTIONS]\n"
              << "Options:\n"
              << "  -h, --help                 Display this help message\n"
              << "  -p, --profile PATH         Path to profile directory (default: ~/Spotlight/default/)\n"
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
    
    std::cerr << "Unknown log level: " << level << ". Using 'info'." << std::endl;
    return spdlog::level::info;
}

CLIOptions parseCLI(int argc, char** argv) {
    CLIOptions options;
    
    // Use default config path
    options.profileDir = "~/Spotlight/default/";
    options.logLevel = spdlog::level::info;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            std::exit(0);
        } else if (arg == "-v" || arg == "--verbose") {
            options.logLevel = spdlog::level::debug;
        } else if (arg == "--verbosity" && i + 1 < argc) {
            options.logLevel = parseLogLevel(argv[++i]);
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            options.profileDir = argv[++i];
        } else if (i == 1 && arg[0] != '-') {
            // Support for positional argument (for backward compatibility)
            options.profileDir = arg;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printHelp(argv[0]);
            std::exit(1);
        }
    }
    
    return options;
}