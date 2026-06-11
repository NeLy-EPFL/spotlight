#pragma once

#include <fstream>
#include <tuple>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

// A linear mapping from two 2D spaces (plus intercept) to 2D space.
// For example, this could represent the mapping from (stage position, pixel
// position) to physical position. The first 2x2 block is the stage block, the
// second 2x2 block is the pixel block, and then there are two bias terms.
class LinearMapper2x2to2 {
  public:
    double w_x1_to_x, w_y1_to_x, w_x2_to_x, w_y2_to_x, bias_x;
    double w_x1_to_y, w_y1_to_y, w_x2_to_y, w_y2_to_y, bias_y;

    LinearMapper2x2to2();
    // Expects canonical YAML format: x/y nodes each with x1/y1/x2/y2/bias keys.
    LinearMapper2x2to2(const YAML::Node &calibration_node);

    std::tuple<double, double>
    map(double x1, double y1, double x2, double y2) const;
};

class CalibrationParams {
  public:
    bool is_defined;

    CalibrationParams();
    CalibrationParams(const std::string &calibration_file_path);
    CalibrationParams(const CalibrationParams &) = delete;
    CalibrationParams &operator=(const CalibrationParams &) = delete;

    LinearMapper2x2to2 &stage_and_pixel_to_physical;
    LinearMapper2x2to2 &stage_and_physical_to_pixel;
    LinearMapper2x2to2 &physical_and_pixel_to_stage;

    std::tuple<double, double> stage_pos_and_pixel_pos_to_physical_pos(
        double stage_pos_x,
        double stage_pos_y,
        int pixel_pos_row,
        int pixel_pos_col) const;
    std::tuple<int, int> stage_pos_and_physical_pos_to_pixel_pos(
        double stage_pos_x,
        double stage_pos_y,
        double physical_pos_x,
        double physical_pos_y) const;
    std::tuple<double, double> physical_pos_and_pixel_pos_to_stage_pos(
        double physical_pos_x,
        double physical_pos_y,
        int pixel_pos_row,
        int pixel_pos_col) const;

    void save_to_file(const std::string &yaml_path) const;

  private:
    YAML::Node calibration_;
    LinearMapper2x2to2 stage_and_pixel_to_physical_;
    LinearMapper2x2to2 stage_and_physical_to_pixel_;
    LinearMapper2x2to2 physical_and_pixel_to_stage_;
};
