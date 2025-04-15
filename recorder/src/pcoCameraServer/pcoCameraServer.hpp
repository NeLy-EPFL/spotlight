#ifndef PCO_LINUX
#define PCO_LINUX
#endif

#ifndef PCO_CAMERA_SERVER_HPP
#define PCO_CAMERA_SERVER_HPP

#include <stdio.h>
#include <string.h>
#include <csignal>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"

#include "sharedMemoryUtils.hpp"
#include "../recorderConfig.hpp"

namespace PCOCameraServer
{
    struct CLIOptions
    {
        std::string profileDir = "~/Spotlight/default/";
        unsigned int imageWidth = -1;
        unsigned int imageHeight = -1;
        int xOffset = -1;
        int yOffset = -1;
        spdlog::level::level_enum logLevel = spdlog::level::info;
    };

    std::atomic<bool> shutdownRequested(false);

    void printHelp(const char *programName);
    spdlog::level::level_enum parseLogLevel(const std::string &level);
    CLIOptions parseCLI(int argc, char **argv);

    void signalHandler(int signal);

    int calculateOffset(int fullFrameSize, int roiSize);
    void setupPCOCamera(pco::Camera &camera,
                        unsigned int defaultExposureTimeUs,
                        unsigned int imageWidth,
                        unsigned int imageHeight,
                        int xOffset,
                        int yOffset,
                        unsigned int fullFrameWidth,
                        unsigned int fullFrameHeight);

    void serveFrames(const std::string &shmFrameDataName,
                     const size_t frameBufferSize,
                     const std::string &shmExposureTimeName,
                     const std::string &shmMutexName,
                     const std::string &shmFrameCountName,
                     const unsigned int defaultExposureTimeUs,
                     const unsigned int imageWidth,
                     const unsigned int imageHeight,
                     const unsigned int fullFrameWidth,
                     const unsigned int fullFrameHeight);
}

#endif // PCO_CAMERA_SERVER_HPP