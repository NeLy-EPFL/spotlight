#include "recorderConfig.hpp"

RecorderConfig::RecorderConfig()
    : isDefined(false)
{
}

RecorderConfig::RecorderConfig(const std::string &yamlPath)
    : isDefined(true)
{
    try
    {
        parameters_ = YAML::LoadFile(yamlPath);
    }
    catch (const YAML::Exception &e)
    {
        spdlog::error("Failed to load parameters file: {}", e.what());
        throw std::runtime_error("Failed to load parameters file");
    }

    // Check if version is compatible
    int majorVersion =
        parameters_["metadata"]["file_format_version"]["major"].as<int>();
    int minorVersion =
        parameters_["metadata"]["file_format_version"]["minor"].as<int>();
    bool isVersionCompatible =
        checkVersionCompatibility(majorVersion,
                                  minorVersion,
                                  RECORDER_CONFIG_MAJOR,
                                  RECORDER_CONFIG_MINOR);
    if (!isVersionCompatible)
    {
        throw std::runtime_error("File version incompatible: " + yamlPath);
    }
}

void RecorderConfig::saveToFile(const std::string &yamlPath)
{
    if (!isDefined)
    {
        throw std::runtime_error("Parameters are not defined.");
    }

    try
    {
        std::ofstream fout(yamlPath);
        fout << parameters_;
    }
    catch (const std::exception &e)
    {
        spdlog::error("Failed to save recorder parameters to file: {}",
                      e.what());
        throw std::runtime_error("Failed to save recorder parameters to file");
    }
}