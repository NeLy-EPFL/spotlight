#pragma once

#include <cassert>
#include <csignal>
#include <functional>
#include <iostream>
#include <string>
#include <tuple>

#include <EGrabber.h>
#include <FormatConverter.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "recorder/common/data_types.h"
#include "recorder/common/utils.h"

class BehaviorCamera {
  public:
    BehaviorCamera(
        unsigned int imageWidth,
        unsigned int imageHeight,
        unsigned int xOffset,
        unsigned int yOffset,
        const std::string &ioLine);
    ~BehaviorCamera();
    void start(size_t bufferSize = 40);
    void stop();
    FrameData waitForOneFrame();
    bool isReady() const;

  private:
    Euresys::EGenTL genTL_;
    Euresys::EGrabberCameraInfo camera_;
    std::unique_ptr<Euresys::EGrabber<>> frameGrabberPtr_;
    std::unique_ptr<Euresys::FormatConverter> formatConverterPtr_;
    int imageWidth_;
    int imageHeight_;
    int xOffset_;
    int yOffset_;
    std::string ioLine_;
    int currentFPS_;
    std::atomic<bool> cameraReadyFlag_{false};

    // Apply the full GenICam configuration to the grabber and camera: the ROI
    // and external-trigger setup plus the base configuration ported from
    // etc/euresys_config.js. Called once by the constructor.
    void configure();

    template <typename Module>
    bool setIntegerAndCheck(const std::string &key, int value);

    template <typename Module>
    bool setStringAndCheck(const std::string &key, const std::string &value);
};

int roundToNearestValidBehaviorCamDimension(int value);

std::tuple<int, int> getCenteredOffsets(
    int imageWidth, int imageHeight, int fullFrameWidth, int fullFrameHeight);

