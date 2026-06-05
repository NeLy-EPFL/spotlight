#include "calibration.hpp"

LinearMapper2x2to2::LinearMapper2x2to2()
    : w_X1toX(0), w_Y1toX(0), w_X2toX(0), w_Y2toX(0), biasX(0), w_X1toY(0),
      w_Y1toY(0), w_X2toY(0), w_Y2toY(0), biasY(0) {}

LinearMapper2x2to2::LinearMapper2x2to2(const YAML::Node &calibrationNode) {
    auto xNode = calibrationNode["x"];
    auto yNode = calibrationNode["y"];
    w_X1toX = xNode["x1"].as<double>();
    w_Y1toX = xNode["y1"].as<double>();
    w_X2toX = xNode["x2"].as<double>();
    w_Y2toX = xNode["y2"].as<double>();
    biasX = xNode["bias"].as<double>();
    w_X1toY = yNode["x1"].as<double>();
    w_Y1toY = yNode["y1"].as<double>();
    w_X2toY = yNode["x2"].as<double>();
    w_Y2toY = yNode["y2"].as<double>();
    biasY = yNode["bias"].as<double>();
}

std::tuple<double, double>
LinearMapper2x2to2::map(double x1, double y1, double x2, double y2) const {
    return {
        w_X1toX * x1 + w_Y1toX * y1 + w_X2toX * x2 + w_Y2toX * y2 + biasX,
        w_X1toY * x1 + w_Y1toY * y1 + w_X2toY * x2 + w_Y2toY * y2 + biasY,
    };
}

CalibrationParams::CalibrationParams()
    : isDefined(false), stageAndPixelToPhysical(stageAndPixelToPhysical_),
      stageAndPhysicalToPixel(stageAndPhysicalToPixel_),
      physicalAndPixelToStage(physicalAndPixelToStage_) {}

CalibrationParams::CalibrationParams(const std::string &calibrationFilePath)
    : isDefined(true), stageAndPixelToPhysical(stageAndPixelToPhysical_),
      stageAndPhysicalToPixel(stageAndPhysicalToPixel_),
      physicalAndPixelToStage(physicalAndPixelToStage_) {
    try {
        calibration_ = YAML::LoadFile(calibrationFilePath);

        if (!calibration_["stage_and_pixel_to_physical"] ||
            !calibration_["stage_and_physical_to_pixel"]) {
            throw std::runtime_error("Missing required calibration sections");
        }
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(
            "Failed to load calibration file: " + std::string(e.what()));
    }

    // Populate stageAndPixelToPhysical_
    // YAML keys: stage_pos_x/y -> first input pair; pixel_pos_x(col)/y(row) ->
    // second
    auto sptp_x = calibration_["stage_and_pixel_to_physical"]["physical_pos_x"];
    auto sptp_y = calibration_["stage_and_pixel_to_physical"]["physical_pos_y"];
    stageAndPixelToPhysical_.w_X1toX = sptp_x["stage_pos_x"].as<double>();
    stageAndPixelToPhysical_.w_Y1toX = sptp_x["stage_pos_y"].as<double>();
    stageAndPixelToPhysical_.w_X2toX = sptp_x["pixel_pos_x"].as<double>();
    stageAndPixelToPhysical_.w_Y2toX = sptp_x["pixel_pos_y"].as<double>();
    stageAndPixelToPhysical_.biasX = sptp_x["bias"].as<double>();
    stageAndPixelToPhysical_.w_X1toY = sptp_y["stage_pos_x"].as<double>();
    stageAndPixelToPhysical_.w_Y1toY = sptp_y["stage_pos_y"].as<double>();
    stageAndPixelToPhysical_.w_X2toY = sptp_y["pixel_pos_x"].as<double>();
    stageAndPixelToPhysical_.w_Y2toY = sptp_y["pixel_pos_y"].as<double>();
    stageAndPixelToPhysical_.biasY = sptp_y["bias"].as<double>();

    // Populate stageAndPhysicalToPixel_
    // X output = pixel_col (pixel_pos_x), Y output = pixel_row (pixel_pos_y)
    auto satp_col = calibration_["stage_and_physical_to_pixel"]["pixel_pos_x"];
    auto satp_row = calibration_["stage_and_physical_to_pixel"]["pixel_pos_y"];
    stageAndPhysicalToPixel_.w_X1toX = satp_col["stage_pos_x"].as<double>();
    stageAndPhysicalToPixel_.w_Y1toX = satp_col["stage_pos_y"].as<double>();
    stageAndPhysicalToPixel_.w_X2toX = satp_col["physical_pos_x"].as<double>();
    stageAndPhysicalToPixel_.w_Y2toX = satp_col["physical_pos_y"].as<double>();
    stageAndPhysicalToPixel_.biasX = satp_col["bias"].as<double>();
    stageAndPhysicalToPixel_.w_X1toY = satp_row["stage_pos_x"].as<double>();
    stageAndPhysicalToPixel_.w_Y1toY = satp_row["stage_pos_y"].as<double>();
    stageAndPhysicalToPixel_.w_X2toY = satp_row["physical_pos_x"].as<double>();
    stageAndPhysicalToPixel_.w_Y2toY = satp_row["physical_pos_y"].as<double>();
    stageAndPhysicalToPixel_.biasY = satp_row["bias"].as<double>();

    // Populate physicalAndPixelToStage_ by analytically inverting the stage
    // block of stageAndPixelToPhysical_. Holding pixel fixed:
    //   [sx]   [a b]^-1   [phys_x - c*col - d*row - biasX]
    //   [sy] = [e f]    * [phys_y - g*col - h*row - biasY]
    double a = stageAndPixelToPhysical_.w_X1toX;
    double b = stageAndPixelToPhysical_.w_Y1toX;
    double c = stageAndPixelToPhysical_.w_X2toX;
    double d = stageAndPixelToPhysical_.w_Y2toX;
    double bX = stageAndPixelToPhysical_.biasX;
    double e = stageAndPixelToPhysical_.w_X1toY;
    double f = stageAndPixelToPhysical_.w_Y1toY;
    double g = stageAndPixelToPhysical_.w_X2toY;
    double h = stageAndPixelToPhysical_.w_Y2toY;
    double bY = stageAndPixelToPhysical_.biasY;
    double det = a * f - b * e;
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error(
            "Calibration stage-block is singular; cannot invert.");
    }
    physicalAndPixelToStage_.w_X1toX = f / det;
    physicalAndPixelToStage_.w_Y1toX = -b / det;
    physicalAndPixelToStage_.w_X2toX = (-f * c + b * g) / det;
    physicalAndPixelToStage_.w_Y2toX = (-f * d + b * h) / det;
    physicalAndPixelToStage_.biasX = (-f * bX + b * bY) / det;
    physicalAndPixelToStage_.w_X1toY = -e / det;
    physicalAndPixelToStage_.w_Y1toY = a / det;
    physicalAndPixelToStage_.w_X2toY = (e * c - a * g) / det;
    physicalAndPixelToStage_.w_Y2toY = (e * d - a * h) / det;
    physicalAndPixelToStage_.biasY = (e * bX - a * bY) / det;
}

std::tuple<double, double> CalibrationParams::stagePosAndPixelPosToPhysicalPos(
    double stagePosX,
    double stagePosY,
    int pixelPosRow,
    int pixelPosCol) const {
    if (!isDefined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    return stageAndPixelToPhysical_.map(
        stagePosX, stagePosY, pixelPosCol, pixelPosRow);
}

std::tuple<int, int> CalibrationParams::stagePosAndPhysicalPosToPixelPos(
    double stagePosX,
    double stagePosY,
    double physicalPosX,
    double physicalPosY) const {
    if (!isDefined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    auto [col, row] = stageAndPhysicalToPixel_.map(
        stagePosX, stagePosY, physicalPosX, physicalPosY);
    return {
        static_cast<int>(std::round(row)), static_cast<int>(std::round(col))};
}

std::tuple<double, double> CalibrationParams::physicalPosAndPixelPosToStagePos(
    double physicalPosX,
    double physicalPosY,
    int pixelPosRow,
    int pixelPosCol) const {
    if (!isDefined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    return physicalAndPixelToStage_.map(
        physicalPosX, physicalPosY, pixelPosCol, pixelPosRow);
}

void CalibrationParams::saveToFile(const std::string &yamlPath) {
    if (!isDefined) {
        throw std::runtime_error("Calibration parameters are not defined.");
    }

    try {
        std::ofstream fout(yamlPath);
        fout << calibration_;
    } catch (const std::exception &e) {
        spdlog::error(
            "Failed to save calibration parameters to file: {}", e.what());
        throw std::runtime_error(
            "Failed to save calibration parameters to file");
    }
}
