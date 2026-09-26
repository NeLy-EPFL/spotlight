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
#include <ctime> // timegm, std::tm
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

#include "recorder/apps/shared_memory_utils.h"
#include "recorder/common/recorder_config.h"

namespace pco_camera_server {
struct CLIOptions {
    std::string camera_role; // "calcium" or "fiducial" (required)
    std::string profile_dir = "~/Spotlight/default/";
    unsigned int x0 = 1;
    unsigned int x1 = 2048;
    unsigned int y0 = 1;
    unsigned int y1 = 2048;
    unsigned int delay_us = 0; // Delay after trigger in microseconds
    spdlog::level::level_enum log_level = spdlog::level::info;
};

std::atomic<bool> shutdown_requested(false);

void print_help(const char *program_name);
spdlog::level::level_enum parse_log_level(const std::string &level);
CLIOptions parse_cli(int argc, char **argv);

void signal_handler(int signal);

// Convert a PCO binary timestamp (camera clock) to microseconds since the Unix
// epoch, interpreting the camera's date and time as UTC.
uint64_t pco_timestamp_to_epoch_us(const PCO_TIMESTAMP_STRUCT &timestamp);

void setup_pco_camera(
    pco::Camera &camera,
    unsigned int default_shutter_open_time_us,
    unsigned int x0,
    unsigned int x1,
    unsigned int y0,
    unsigned int y1,
    unsigned int delay_us,
    unsigned int full_frame_width,
    unsigned int full_frame_height);

// Create the shared-memory regions named in `shm_names`, open the PCO camera
// with `serial_number`, and publish its frames until shutdown is requested.
void serve_frames(
    const pco_shared_memory::SharedMemoryNames &shm_names,
    const DWORD serial_number,
    const size_t frame_buffer_size,
    const unsigned int default_shutter_open_time_us,
    const unsigned int x0,
    const unsigned int x1,
    const unsigned int y0,
    const unsigned int y1,
    const unsigned int delay_us,
    const unsigned int full_frame_width,
    const unsigned int full_frame_height);
} // namespace pco_camera_server
