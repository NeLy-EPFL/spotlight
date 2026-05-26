#ifndef RECORDER_CONFIG_HPP
#define RECORDER_CONFIG_HPP

#include <filesystem>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "calibration.hpp"
#include "dataTypes.hpp"

class RecorderConfig {
  public:
    bool isDefined;

    RecorderConfig();
    RecorderConfig(const std::string &yamlPath);

    // Add const qualifier to make it usable with const objects
    template <typename T>
    T getParameter(const std::string &section, const std::string &parameter) const;

    void saveToFile(const std::string &yamlPath);

  private:
    YAML::Node parameters_;
};

// Template implementation must be in the header file
template <typename T>
T RecorderConfig::getParameter(const std::string &section, const std::string &parameter) const {
    if (!isDefined) {
        throw std::runtime_error("Parameters are not loaded yet.");
    }

    try {
        return parameters_[section][parameter].as<T>();
    } catch (const YAML::Exception &e) {
        // Use spdlog to match your include
        spdlog::error("Failed to get parameter: {}.{} - {}", section, parameter, e.what());
        throw std::runtime_error("Failed to get parameter");
    }
}

class ActiveAreaMask {
  public:
    cv::Mat fullArenaMask;
    double resolutionMmPerPixel;
    double arenaWidthMm;
    double arenaHeightMm;
    LinearMapper2x2to2 &stageAndPixelToPhysical;

    ActiveAreaMask(const std::string &arenaSpecDir, double boundaryMarginMm,
                   LinearMapper2x2to2 &stageAndPixelToPhysical);
    cv::Mat warpToCurrentView(cv::Mat currentImage, MotionStagePosition stagePos);

  private:
    cv::Mat transformMatrixAtZeroStagePos_;
};

#endif // RECORDER_CONFIG_HPP