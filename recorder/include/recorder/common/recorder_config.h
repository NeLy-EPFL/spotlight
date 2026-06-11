#pragma once

#include <filesystem>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/calibration.h"
#include "recorder/common/data_types.h"

class RecorderConfig {
  public:
    bool is_defined;

    RecorderConfig();
    RecorderConfig(const std::string &yaml_path);

    // Add const qualifier to make it usable with const objects
    template <typename T>
    T get_parameter(
        const std::string &section, const std::string &parameter) const;

    void save_to_file(const std::string &yaml_path) const;

  private:
    YAML::Node parameters_;
};

// Template implementation must be in the header file
template <typename T>
T RecorderConfig::get_parameter(
    const std::string &section, const std::string &parameter) const {
    if (!is_defined) {
        throw std::runtime_error("Parameters are not loaded yet.");
    }

    try {
        return parameters_[section][parameter].as<T>();
    } catch (const YAML::Exception &e) {
        // Use spdlog to match your include
        spdlog::error(
            "Failed to get parameter: {}.{} - {}",
            section,
            parameter,
            e.what());
        throw std::runtime_error("Failed to get parameter");
    }
}
