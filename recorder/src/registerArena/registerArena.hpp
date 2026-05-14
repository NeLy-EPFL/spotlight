#ifndef REGISTER_ARENA_HPP
#define REGISTER_ARENA_HPP

#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>

#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "../common/behaviorRecording.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/motionControl.hpp"
#include "../peripherals/arduinoCommunication.hpp"

void registerArena(std::filesystem::path profileDir, std::string arenaName);

#endif // REGISTER_ARENA_HPP
