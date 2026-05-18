#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <tuple>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

class CalibrationParams
{
public:
    bool isDefined;

    CalibrationParams();
    CalibrationParams(const std::string &calibrationFilePath);

    std::tuple<double, double> stagePosAndPixelPosToPhysicalPos(
        double stagePosX,
        double stagePosY,
        int pixelPosRow,
        int pixelPosCol) const;
    std::tuple<int, int> stagePosAndPhysicalPosToPixelPos(
        double stagePosX,
        double stagePosY,
        double physicalPosX,
        double physicalPosY) const;
    // Invert the calibration model at a fixed pixel: given a physical
    // (arena) position and a pixel coordinate, return the stage position
    // at which the camera, looking at that pixel, would see that arena
    // point. This is a 2x2 linear solve, so the stage block of the
    // forward model must be non-singular.
    std::tuple<double, double> physicalPosAndPixelPosToStagePos(
        double physicalPosX,
        double physicalPosY,
        int pixelPosRow,
        int pixelPosCol) const;

    void saveToFile(const std::string &yamlPath);

private:
    YAML::Node calibration_;
};

#endif // CALIBRATION_HPP