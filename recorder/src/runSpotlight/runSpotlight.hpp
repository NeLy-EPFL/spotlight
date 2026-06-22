#ifndef MAIN_HPP
#define MAIN_HPP

#include <QApplication>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <signal.h>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>
#include <vector>

#include "../common/behaviorRecording.hpp"
#include "../common/calibration.hpp"
#include "../common/cli.hpp"
#include "../common/muscleRecording.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/trackingControl.hpp"
#include "../peripherals/arduinoCommunication.hpp"
#include "../peripherals/behaviorCamera.hpp"
#include "gui.hpp"

// Forward declarations
class QApplication;

// Program control functions
bool quitProgram();

#endif // MAIN_HPP