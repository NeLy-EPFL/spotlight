#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <tuple>

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

private:
    YAML::Node calibration_;

    void loadCalibration(const std::string &filename);
};

#endif // CALIBRATION_HPP