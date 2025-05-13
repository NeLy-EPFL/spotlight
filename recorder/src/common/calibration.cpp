#include "calibration.hpp"

CalibrationParams::CalibrationParams()
    : isDefined(false)
{
}

CalibrationParams::CalibrationParams(const std::string &calibrationFilePath)
    : isDefined(true)
{
    // spdlog::critical("trying to load")
    try
    {
        calibration_ = YAML::LoadFile(calibrationFilePath);

        // Check if version is compatible
        int majorVersion =
            calibration_["metadata"]["file_format_version"]["major"].as<int>();
        int minorVersion =
            calibration_["metadata"]["file_format_version"]["minor"].as<int>();
        bool isVersionCompatible =
            checkVersionCompatibility(majorVersion,
                                      minorVersion,
                                      CALIBRATION_RESULT_MAJOR,
                                      CALIBRATION_RESULT_MINOR);
        if (!isVersionCompatible)
        {
            throw std::runtime_error(
                "File version incompatible: " + calibrationFilePath);
        }

        // Validate that all required sections exist
        if (!calibration_["stage_and_pixel_to_physical"] ||
            !calibration_["stage_and_physical_to_pixel"])
        {
            throw std::runtime_error("Missing required calibration sections");
        }
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error(
            "Failed to load calibration file: " + std::string(e.what()));
    }
}

std::tuple<double, double>
CalibrationParams::stagePosAndPixelPosToPhysicalPos(
    double stagePosX,
    double stagePosY,
    int pixelPosRow,
    int pixelPosCol) const
{
    // Check if calibration data is loaded
    if (!calibration_.IsDefined())
    {
        throw std::runtime_error("Calibration data not loaded");
    }

    // Access the calibration parameters for physical_pos_x
    auto physicalPosXParams =
        calibration_["stage_and_pixel_to_physical"]["physical_pos_x"];
    double physicalPosX =
        physicalPosXParams["stage_pos_x"].as<double>() * stagePosX +
        physicalPosXParams["stage_pos_y"].as<double>() * stagePosY +
        physicalPosXParams["pixel_pos_x"].as<double>() * pixelPosCol +
        physicalPosXParams["pixel_pos_y"].as<double>() * pixelPosRow +
        physicalPosXParams["bias"].as<double>();

    // Access the calibration parameters for physical_pos_y
    auto physicalPosYParams =
        calibration_["stage_and_pixel_to_physical"]["physical_pos_y"];
    double physicalPosY =
        physicalPosYParams["stage_pos_x"].as<double>() * stagePosX +
        physicalPosYParams["stage_pos_y"].as<double>() * stagePosY +
        physicalPosYParams["pixel_pos_x"].as<double>() * pixelPosCol +
        physicalPosYParams["pixel_pos_y"].as<double>() * pixelPosRow +
        physicalPosYParams["bias"].as<double>();

    return std::make_tuple(physicalPosX, physicalPosY);
}

std::tuple<int, int>
CalibrationParams::stagePosAndPhysicalPosToPixelPos(
    double stagePosX,
    double stagePosY,
    double physicalPosX,
    double physicalPosY) const
{
    // Check if calibration data is loaded
    if (!calibration_.IsDefined())
    {
        throw std::runtime_error("Calibration data not loaded");
    }

    // Access the calibration parameters for pixel_pos_row
    auto pixelPosRowParams =
        calibration_["stage_and_physical_to_pixel"]["pixel_pos_y"];
    double pixelPosRow =
        pixelPosRowParams["stage_pos_x"].as<double>() * stagePosX +
        pixelPosRowParams["stage_pos_y"].as<double>() * stagePosY +
        pixelPosRowParams["physical_pos_x"].as<double>() * physicalPosX +
        pixelPosRowParams["physical_pos_y"].as<double>() * physicalPosY +
        pixelPosRowParams["bias"].as<double>();

    // Access the calibration parameters for pixel_pos_col
    auto pixelPosColParams =
        calibration_["stage_and_physical_to_pixel"]["pixel_pos_x"];
    double pixelPosCol =
        pixelPosColParams["stage_pos_x"].as<double>() * stagePosX +
        pixelPosColParams["stage_pos_y"].as<double>() * stagePosY +
        pixelPosColParams["physical_pos_x"].as<double>() * physicalPosX +
        pixelPosColParams["physical_pos_y"].as<double>() * physicalPosY +
        pixelPosColParams["bias"].as<double>();

    return std::make_tuple(static_cast<int>(std::round(pixelPosRow)),
                           static_cast<int>(std::round(pixelPosCol)));
}

void CalibrationParams::saveToFile(const std::string &yamlPath)
{
    if (!isDefined)
    {
        throw std::runtime_error("Calibration parameters are not defined.");
    }

    try
    {
        std::ofstream fout(yamlPath);
        fout << calibration_;
    }
    catch (const std::exception &e)
    {
        spdlog::error("Failed to save calibration parameters to file: {}",
                      e.what());
        throw std::runtime_error(
            "Failed to save calibration parameters to file");
    }
}