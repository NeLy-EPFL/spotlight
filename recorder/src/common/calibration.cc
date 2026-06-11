#include "recorder/common/calibration.h"

LinearMapper2x2to2::LinearMapper2x2to2()
    : w_x1_to_x(0), w_y1_to_x(0), w_x2_to_x(0), w_y2_to_x(0), bias_x(0),
      w_x1_to_y(0), w_y1_to_y(0), w_x2_to_y(0), w_y2_to_y(0), bias_y(0) {}

LinearMapper2x2to2::LinearMapper2x2to2(const YAML::Node &calibration_node) {
    auto x_node = calibration_node["x"];
    auto y_node = calibration_node["y"];
    w_x1_to_x = x_node["x1"].as<double>();
    w_y1_to_x = x_node["y1"].as<double>();
    w_x2_to_x = x_node["x2"].as<double>();
    w_y2_to_x = x_node["y2"].as<double>();
    bias_x = x_node["bias"].as<double>();
    w_x1_to_y = y_node["x1"].as<double>();
    w_y1_to_y = y_node["y1"].as<double>();
    w_x2_to_y = y_node["x2"].as<double>();
    w_y2_to_y = y_node["y2"].as<double>();
    bias_y = y_node["bias"].as<double>();
}

std::tuple<double, double>
LinearMapper2x2to2::map(double x1, double y1, double x2, double y2) const {
    return {
        w_x1_to_x * x1 + w_y1_to_x * y1 + w_x2_to_x * x2 + w_y2_to_x * y2 +
            bias_x,
        w_x1_to_y * x1 + w_y1_to_y * y1 + w_x2_to_y * x2 + w_y2_to_y * y2 +
            bias_y,
    };
}

CalibrationParams::CalibrationParams()
    : is_defined(false),
      stage_and_pixel_to_physical(stage_and_pixel_to_physical_),
      stage_and_physical_to_pixel(stage_and_physical_to_pixel_),
      physical_and_pixel_to_stage(physical_and_pixel_to_stage_) {}

CalibrationParams::CalibrationParams(const std::string &calibration_file_path)
    : is_defined(true),
      stage_and_pixel_to_physical(stage_and_pixel_to_physical_),
      stage_and_physical_to_pixel(stage_and_physical_to_pixel_),
      physical_and_pixel_to_stage(physical_and_pixel_to_stage_) {
    try {
        calibration_ = YAML::LoadFile(calibration_file_path);

        if (!calibration_["stage_and_pixel_to_physical"] ||
            !calibration_["stage_and_physical_to_pixel"]) {
            throw std::runtime_error("Missing required calibration sections");
        }
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(
            "Failed to load calibration file: " + std::string(e.what()));
    }

    // Populate stage_and_pixel_to_physical_
    // YAML keys: stage_pos_x/y -> first input pair; pixel_pos_x(col)/y(row) ->
    // second
    auto sptp_x = calibration_["stage_and_pixel_to_physical"]["physical_pos_x"];
    auto sptp_y = calibration_["stage_and_pixel_to_physical"]["physical_pos_y"];
    stage_and_pixel_to_physical_.w_x1_to_x = sptp_x["stage_pos_x"].as<double>();
    stage_and_pixel_to_physical_.w_y1_to_x = sptp_x["stage_pos_y"].as<double>();
    stage_and_pixel_to_physical_.w_x2_to_x = sptp_x["pixel_pos_x"].as<double>();
    stage_and_pixel_to_physical_.w_y2_to_x = sptp_x["pixel_pos_y"].as<double>();
    stage_and_pixel_to_physical_.bias_x = sptp_x["bias"].as<double>();
    stage_and_pixel_to_physical_.w_x1_to_y = sptp_y["stage_pos_x"].as<double>();
    stage_and_pixel_to_physical_.w_y1_to_y = sptp_y["stage_pos_y"].as<double>();
    stage_and_pixel_to_physical_.w_x2_to_y = sptp_y["pixel_pos_x"].as<double>();
    stage_and_pixel_to_physical_.w_y2_to_y = sptp_y["pixel_pos_y"].as<double>();
    stage_and_pixel_to_physical_.bias_y = sptp_y["bias"].as<double>();

    // Populate stage_and_physical_to_pixel_
    // X output = pixel_col (pixel_pos_x), Y output = pixel_row (pixel_pos_y)
    auto satp_col = calibration_["stage_and_physical_to_pixel"]["pixel_pos_x"];
    auto satp_row = calibration_["stage_and_physical_to_pixel"]["pixel_pos_y"];
    stage_and_physical_to_pixel_.w_x1_to_x =
        satp_col["stage_pos_x"].as<double>();
    stage_and_physical_to_pixel_.w_y1_to_x =
        satp_col["stage_pos_y"].as<double>();
    stage_and_physical_to_pixel_.w_x2_to_x =
        satp_col["physical_pos_x"].as<double>();
    stage_and_physical_to_pixel_.w_y2_to_x =
        satp_col["physical_pos_y"].as<double>();
    stage_and_physical_to_pixel_.bias_x = satp_col["bias"].as<double>();
    stage_and_physical_to_pixel_.w_x1_to_y =
        satp_row["stage_pos_x"].as<double>();
    stage_and_physical_to_pixel_.w_y1_to_y =
        satp_row["stage_pos_y"].as<double>();
    stage_and_physical_to_pixel_.w_x2_to_y =
        satp_row["physical_pos_x"].as<double>();
    stage_and_physical_to_pixel_.w_y2_to_y =
        satp_row["physical_pos_y"].as<double>();
    stage_and_physical_to_pixel_.bias_y = satp_row["bias"].as<double>();

    // Populate physical_and_pixel_to_stage_ by analytically inverting the stage
    // block of stage_and_pixel_to_physical_. Holding pixel fixed:
    //   [sx]   [a b]^-1   [phys_x - c*col - d*row - bias_x]
    //   [sy] = [e f]    * [phys_y - g*col - h*row - bias_y]
    double a = stage_and_pixel_to_physical_.w_x1_to_x;
    double b = stage_and_pixel_to_physical_.w_y1_to_x;
    double c = stage_and_pixel_to_physical_.w_x2_to_x;
    double d = stage_and_pixel_to_physical_.w_y2_to_x;
    double b_x = stage_and_pixel_to_physical_.bias_x;
    double e = stage_and_pixel_to_physical_.w_x1_to_y;
    double f = stage_and_pixel_to_physical_.w_y1_to_y;
    double g = stage_and_pixel_to_physical_.w_x2_to_y;
    double h = stage_and_pixel_to_physical_.w_y2_to_y;
    double b_y = stage_and_pixel_to_physical_.bias_y;
    double det = a * f - b * e;
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error(
            "Calibration stage-block is singular; cannot invert.");
    }
    physical_and_pixel_to_stage_.w_x1_to_x = f / det;
    physical_and_pixel_to_stage_.w_y1_to_x = -b / det;
    physical_and_pixel_to_stage_.w_x2_to_x = (-f * c + b * g) / det;
    physical_and_pixel_to_stage_.w_y2_to_x = (-f * d + b * h) / det;
    physical_and_pixel_to_stage_.bias_x = (-f * b_x + b * b_y) / det;
    physical_and_pixel_to_stage_.w_x1_to_y = -e / det;
    physical_and_pixel_to_stage_.w_y1_to_y = a / det;
    physical_and_pixel_to_stage_.w_x2_to_y = (e * c - a * g) / det;
    physical_and_pixel_to_stage_.w_y2_to_y = (e * d - a * h) / det;
    physical_and_pixel_to_stage_.bias_y = (e * b_x - a * b_y) / det;
}

std::tuple<double, double>
CalibrationParams::stage_pos_and_pixel_pos_to_physical_pos(
    double stage_pos_x,
    double stage_pos_y,
    int pixel_pos_row,
    int pixel_pos_col) const {
    if (!is_defined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    return stage_and_pixel_to_physical_.map(
        stage_pos_x, stage_pos_y, pixel_pos_col, pixel_pos_row);
}

std::tuple<int, int> CalibrationParams::stage_pos_and_physical_pos_to_pixel_pos(
    double stage_pos_x,
    double stage_pos_y,
    double physical_pos_x,
    double physical_pos_y) const {
    if (!is_defined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    auto [col, row] = stage_and_physical_to_pixel_.map(
        stage_pos_x, stage_pos_y, physical_pos_x, physical_pos_y);
    return {
        static_cast<int>(std::round(row)), static_cast<int>(std::round(col))};
}

std::tuple<double, double>
CalibrationParams::physical_pos_and_pixel_pos_to_stage_pos(
    double physical_pos_x,
    double physical_pos_y,
    int pixel_pos_row,
    int pixel_pos_col) const {
    if (!is_defined) {
        throw std::runtime_error("Calibration data not loaded");
    }
    return physical_and_pixel_to_stage_.map(
        physical_pos_x, physical_pos_y, pixel_pos_col, pixel_pos_row);
}

void CalibrationParams::save_to_file(const std::string &yaml_path) const {
    if (!is_defined) {
        throw std::runtime_error("Calibration parameters are not defined.");
    }

    try {
        std::ofstream fout(yaml_path);
        fout << calibration_;
    } catch (const std::exception &e) {
        spdlog::error(
            "Failed to save calibration parameters to file: {}", e.what());
        throw std::runtime_error(
            "Failed to save calibration parameters to file");
    }
}
