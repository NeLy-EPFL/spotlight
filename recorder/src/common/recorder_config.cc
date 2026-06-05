#include "recorder/common/recorder_config.h"

RecorderConfig::RecorderConfig() : isDefined(false) {}

RecorderConfig::RecorderConfig(const std::string &yamlPath) : isDefined(true) {
    try {
        parameters_ = YAML::LoadFile(yamlPath);
    } catch (const YAML::Exception &e) {
        spdlog::error("Failed to load parameters file: {}", e.what());
        throw std::runtime_error("Failed to load parameters file");
    }
}

void RecorderConfig::saveToFile(const std::string &yamlPath) {
    if (!isDefined) {
        throw std::runtime_error("Parameters are not defined.");
    }

    try {
        std::ofstream fout(yamlPath);
        fout << parameters_;
    } catch (const std::exception &e) {
        spdlog::error(
            "Failed to save recorder parameters to file: {}", e.what());
        throw std::runtime_error("Failed to save recorder parameters to file");
    }
}