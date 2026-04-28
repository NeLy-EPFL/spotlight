#ifndef RECORDER_CONFIG_HPP
#define RECORDER_CONFIG_HPP

#include <fstream>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>
#include <filesystem>

#include "fileFormatVersions.hpp"

class RecorderConfig
{
public:
    bool isDefined;
    std::filesystem::path profileDir;

    RecorderConfig();
    RecorderConfig(const std::string &yamlPath);

    // Add const qualifier to make it usable with const objects
    template <typename T>
    T getParameter(const std::string &section, const std::string &parameter) const;
    
    void saveToFile(const std::string &yamlPath);

private:
    YAML::Node parameters_;;
};

// Template implementation must be in the header file
template <typename T>
T RecorderConfig::getParameter(const std::string &section, const std::string &parameter) const
{
    if (!isDefined)
    {
        throw std::runtime_error("Parameters are not loaded yet.");
    }

    try
    {
        return parameters_[section][parameter].as<T>();
    }
    catch (const YAML::Exception &e)
    {
        // Use spdlog to match your include
        spdlog::error("Failed to get parameter: {}.{} - {}", 
                      section, parameter, e.what());
        throw std::runtime_error("Failed to get parameter");
    }
}

#endif // RECORDER_CONFIG_HPP