#ifndef PCO_LINUX
#define PCO_LINUX
#endif

#ifndef MUSCLE_CAMERA_HPP
#define MUSCLE_CAMERA_HPP

#include <atomic>

#include <spdlog/spdlog.h>
#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"
#include "opencv2/opencv.hpp"

#include "../dataTypes.hpp"
#include "../utils.hpp"

class MuscleCamera
{
public:
    MuscleCamera(double exposureTime,
                 int imageWidth,
                 int imageHeight,
                 int xOffset,
                 int yOffset);
    ~MuscleCamera();
    void start(unsigned int bufferCount = 20);
    void stop();
    FrameData waitForOneFrame();
    bool isReady() const;

private:
    unsigned int imageWidth_;
    unsigned int imageHeight_;
    unsigned int xOffset_;
    unsigned int yOffset_;
    pco::Camera camera_;
    bool firstFrameHasArrived_;
    pco::Image currentPCOImage_;
    std::atomic<bool> cameraReadyFlag_{false}; // atomic for thread safety
};

int roundToNearestValidMuscleCamWidth(int initialValue);
int roundToNearestValidMuscleCamHeight(int initialValue);

#endif // MUSCLE_CAMERA_HPP