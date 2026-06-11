#include "recorder/common/recorder_config.h"

RecorderConfig::RecorderConfig() : is_defined(false) {}

RecorderConfig::RecorderConfig(const std::string &yaml_path)
    : is_defined(true) {
    try {
        parameters_ = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &e) {
        spdlog::error("Failed to load parameters file: {}", e.what());
        throw std::runtime_error("Failed to load parameters file");
    }
}

void RecorderConfig::save_to_file(const std::string &yaml_path) const {
    if (!is_defined) {
        throw std::runtime_error("Parameters are not defined.");
    }

    try {
        std::ofstream fout(yaml_path);
        fout << parameters_;
    } catch (const std::exception &e) {
        spdlog::error(
            "Failed to save recorder parameters to file: {}", e.what());
        throw std::runtime_error("Failed to save recorder parameters to file");
    }
}