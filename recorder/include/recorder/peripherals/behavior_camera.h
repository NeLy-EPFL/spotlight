#pragma once

#include <atomic>
#include <cassert>
#include <csignal>
#include <functional>
#include <iostream>
#include <optional>
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
    // Stop streaming and interrupt any in-progress (or future) waitForOneFrame()
    // so it returns std::nullopt. Thread-safe and idempotent; called from the
    // acquirer at loop exit and from the shutdown path. Does NOT release the
    // grabber device -- that happens in ~BehaviorCamera/~EGrabber.
    void stop();
    // Block until the next frame is acquired and return it. Returns std::nullopt
    // once stop() has been called (so the acquirer loop can exit promptly even if
    // no frames are arriving -- e.g. the camera is not being triggered). The grab
    // uses a bounded timeout and cancelPop() so it never blocks indefinitely.
    std::optional<FrameData> waitForOneFrame();
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
    // Set by stop(); makes waitForOneFrame() return std::nullopt so the acquirer
    // thread can be joined on shutdown.
    std::atomic<bool> stopRequested_{false};

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

