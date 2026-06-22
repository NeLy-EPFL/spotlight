/**
 * runArenaRegistrationScan.hpp
 *
 * Declares runArenaRegistrationScan(profileDir, arenaDir), which performs
 * the automated mapping-board scan.  See runArenaRegistrationScan_main.cpp
 * for the full workflow description and output_format.md for the output spec.
 */
#ifndef RUN_ARENA_REGISTRATION_SCAN_HPP
#define RUN_ARENA_REGISTRATION_SCAN_HPP

#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>

#include "../common/behaviorRecording.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "../peripherals/arduinoCommunication.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/motionControl.hpp"

void runArenaRegistrationScan(
    std::filesystem::path profileDir, std::filesystem::path arenaDir);

#endif // RUN_ARENA_REGISTRATION_SCAN_HPP
