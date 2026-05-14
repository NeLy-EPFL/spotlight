#ifndef RUN_ARENA_REGISTRATION_SCAN_HPP
#define RUN_ARENA_REGISTRATION_SCAN_HPP

#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>

#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "../common/behaviorRecording.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/motionControl.hpp"
#include "../peripherals/arduinoCommunication.hpp"

void runArenaRegistrationScan(std::filesystem::path profileDir,
                              std::filesystem::path arenaDir);

#endif // RUN_ARENA_REGISTRATION_SCAN_HPP
