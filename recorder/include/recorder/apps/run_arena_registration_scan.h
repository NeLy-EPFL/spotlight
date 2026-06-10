#pragma once

// Declares runArenaRegistrationScan(profileDir, arenaDir), which performs the
// automated mapping-board scan. See run_arena_registration_scan_main.cc for the
// full workflow description and docs/recorder/run_arena_registration_scan.md for
// the output spec.

#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>

#include "recorder/common/behavior_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/peripherals/motion_control.h"

void runArenaRegistrationScan(
    const std::filesystem::path &profileDir,
    const std::filesystem::path &arenaDir);

