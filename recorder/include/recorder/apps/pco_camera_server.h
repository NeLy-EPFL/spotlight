#pragma once

// The PCO headers select their Linux code paths with `#elif PCO_LINUX`, so
// PCO_LINUX must expand to a non-empty token (an empty define would produce
// `#elif` with no expression). The build normally provides it per-target via
// target_compile_definitions on pco-camera-server (needed by the bundled PCO
// SDK .cpp files, which do not include this header); this is a fallback.
#ifndef PCO_LINUX
#define PCO_LINUX 1
#endif

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
#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"
// clang-format on

#include "recorder/common/recorder_config.h"
#include "recorder/apps/shared_memory_utils.h"

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

void setupPCOCamera(
    pco::Camera &camera,
    unsigned int defaultShutterOpenTimeUs,
    unsigned int x0,
    unsigned int x1,
    unsigned int y0,
    unsigned int y1,
    unsigned int delayUs,
    unsigned int fullFrameWidth,
    unsigned int fullFrameHeight);

void serveFrames(
    const std::string &shmFrameDataName,
    const size_t frameBufferSize,
    const std::string &shmShutterOpenTimeName,
    const std::string &shmFrameMetadataName,
    const std::string &shmMutexName,
    const std::string &shmCondVarName,
    const unsigned int defaultShutterOpenTimeUs,
    const unsigned int x0,
    const unsigned int x1,
    const unsigned int y0,
    const unsigned int y1,
    const unsigned int delayUs,
    const unsigned int fullFrameWidth,
    const unsigned int fullFrameHeight);
} // namespace PCOCameraServer
