#pragma once

#include <QApplication>
#include <QTimer>
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

#include "recorder/common/behavior_recording.h"
#include "recorder/common/calibration.h"
#include "recorder/common/cli.h"
#include "recorder/common/muscle_recording.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/tracking_control.h"
#include "recorder/peripherals/arduino_communication.h"
#include "recorder/peripherals/behavior_camera.h"
#include "recorder/apps/gui.h"

// Forward declarations
class QApplication;

// Program control functions
bool quitProgram();
