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

std::tuple<double, double>
CalibrationParams::physicalPosAndPixelPosToStagePos(
    double physicalPosX,
    double physicalPosY,
    int pixelPosRow,
    int pixelPosCol) const
{
    if (!calibration_.IsDefined())
    {
        throw std::runtime_error("Calibration data not loaded");
    }

    // Forward model:
    //   physical_x = a*sx + b*sy + c*pixCol + d*pixRow + bias_x
    //   physical_y = e*sx + f*sy + g*pixCol + h*pixRow + bias_y
    // Holding pixel fixed, solve the 2x2 system [[a, b], [e, f]] [sx; sy] = rhs.
    auto px = calibration_["stage_and_pixel_to_physical"]["physical_pos_x"];
    auto py = calibration_["stage_and_pixel_to_physical"]["physical_pos_y"];
    double a = px["stage_pos_x"].as<double>();
    double b = px["stage_pos_y"].as<double>();
    double c = px["pixel_pos_x"].as<double>();
    double d = px["pixel_pos_y"].as<double>();
    double biasX = px["bias"].as<double>();
    double e = py["stage_pos_x"].as<double>();
    double f = py["stage_pos_y"].as<double>();
    double g = py["pixel_pos_x"].as<double>();
    double h = py["pixel_pos_y"].as<double>();
    double biasY = py["bias"].as<double>();

    double rhsX = physicalPosX - c * pixelPosCol - d * pixelPosRow - biasX;
    double rhsY = physicalPosY - g * pixelPosCol - h * pixelPosRow - biasY;

    double det = a * f - b * e;
    if (std::abs(det) < 1e-12)
    {
        throw std::runtime_error(
            "Calibration stage-block is singular; cannot invert.");
    }

    double sx = (f * rhsX - b * rhsY) / det;
    double sy = (a * rhsY - e * rhsX) / det;
    return std::make_tuple(sx, sy);
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