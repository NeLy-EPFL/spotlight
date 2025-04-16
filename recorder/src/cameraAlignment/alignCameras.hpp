#ifndef ALIGN_CAMERA_HPP
#define ALIGN_CAMERA_HPP

#include <filesystem>
#include <thread>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/muscleCamera.hpp"
#include "../behaviorRecording.hpp"
#include "../muscleRecording.hpp"
#include "../utils.hpp"
#include "../recorderConfig.hpp"
#include "../dataTypes.hpp"
#include "../cli.hpp"

class PCOCameraROI
{
public:
    int x0;
    int x1;
    int y0;
    int y1;
    int xCenter;
    int yCenter;

    PCOCameraROI(int x0, int x1, int y0, int y1, int xCenter, int yCenter);
    bool isWithinBound(int fullWidth, int fullHeight);
    int toFile(std::filesystem::path path);
};

void alignCamera(std::filesystem::path profileDir);

#endif // ALIGN_CAMERA_HPP