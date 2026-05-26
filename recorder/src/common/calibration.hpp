#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <fstream>
#include <tuple>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief A linear mapping from two 2D spaces (plus intercept) to 2D space.
 * For example, this could represent the mapping from (stage position, pixel
 * position) to physical position. The first 2x2 block is the stage block, the
 * second 2x2 block is the pixel block, and then there are two bias terms.
 */
class LinearMapper2x2to2 {
  public:
    double w_X1toX, w_Y1toX, w_X2toX, w_Y2toX, biasX;
    double w_X1toY, w_Y1toY, w_X2toY, w_Y2toY, biasY;

    LinearMapper2x2to2();
    // Expects canonical YAML format: x/y nodes each with x1/y1/x2/y2/bias keys.
    LinearMapper2x2to2(const YAML::Node &calibrationNode);

    std::tuple<double, double> map(double x1, double y1, double x2, double y2) const;
};

class CalibrationParams {
  public:
    bool isDefined;

    CalibrationParams();
    CalibrationParams(const std::string &calibrationFilePath);
    CalibrationParams(const CalibrationParams &) = delete;
    CalibrationParams &operator=(const CalibrationParams &) = delete;

    LinearMapper2x2to2 &stageAndPixelToPhysical;
    LinearMapper2x2to2 &stageAndPhysicalToPixel;
    LinearMapper2x2to2 &physicalAndPixelToStage;

    std::tuple<double, double> stagePosAndPixelPosToPhysicalPos(
        double stagePosX, double stagePosY, int pixelPosRow, int pixelPosCol) const;
    std::tuple<int, int> stagePosAndPhysicalPosToPixelPos(
        double stagePosX, double stagePosY, double physicalPosX, double physicalPosY) const;
    std::tuple<double, double> physicalPosAndPixelPosToStagePos(
        double physicalPosX, double physicalPosY, int pixelPosRow, int pixelPosCol) const;

    void saveToFile(const std::string &yamlPath);

  private:
    YAML::Node calibration_;
    LinearMapper2x2to2 stageAndPixelToPhysical_;
    LinearMapper2x2to2 stageAndPhysicalToPixel_;
    LinearMapper2x2to2 physicalAndPixelToStage_;
};

#endif // CALIBRATION_HPP