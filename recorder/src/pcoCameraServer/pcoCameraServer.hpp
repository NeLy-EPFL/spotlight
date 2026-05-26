#ifndef PCO_LINUX
#define PCO_LINUX
#endif

#ifndef PCO_CAMERA_SERVER_HPP
#define PCO_CAMERA_SERVER_HPP

#define WAIT_WITH_SMALL_DELAY true
#define WAIT_TIMEOUT_SECS 0.1
#define TIMEOUT_ERROR_CODE 0x80004001 // see PCO manual

#include <atomic>
#include <chrono>
#include <csignal>
#include <stdio.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

// clang-format off
// stdafx.h must come first: it includes pco_linux_defs.h (WORD/BYTE/DWORD) and
// <variant>, which camera.h, cameraexception.h, and sc2_defs.h all depend on.
// Disable clang-format, which sorts includes alphabetically 
#include "stdafx.h"
// clang-format on

#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"

#include "../common/recorderConfig.hpp"
#include "sharedMemoryUtils.hpp"

namespace PCOCameraServer {
struct CLIOptions {
    std::string profileDir = "~/Spotlight/default/";
    unsigned int x0 = 1;
    unsigned int x1 = 2048;
    unsigned int y0 = 1;
    unsigned int y1 = 2048;
    unsigned int delayUs = 0; // Delay after trigger in microseconds
    spdlog::level::level_enum logLevel = spdlog::level::info;
};

std::atomic<bool> shutdownRequested(false);

void printHelp(const char *programName);
spdlog::level::level_enum parseLogLevel(const std::string &level);
CLIOptions parseCLI(int argc, char **argv);

void signalHandler(int signal);

void setupPCOCamera(pco::Camera &camera, unsigned int defaultShutterOpenTimeUs, unsigned int x0,
                    unsigned int x1, unsigned int y0, unsigned int y1, unsigned int delayUs,
                    unsigned int fullFrameWidth, unsigned int fullFrameHeight);

void serveFrames(const std::string &shmFrameDataName, const size_t frameBufferSize,
                 const std::string &shmShutterOpenTimeName, const std::string &shmFrameMetadataName,
                 const std::string &shmMutexName, const std::string &shmCondVarName,
                 const unsigned int defaultShutterOpenTimeUs, const unsigned int x0,
                 const unsigned int x1, const unsigned int y0, const unsigned int y1,
                 const unsigned int delayUs, const unsigned int fullFrameWidth,
                 const unsigned int fullFrameHeight);
} // namespace PCOCameraServer

#endif // PCO_CAMERA_SERVER_HPP