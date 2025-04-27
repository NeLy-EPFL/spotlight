#ifndef MAIN_HPP
#define MAIN_HPP

#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>

#include "../peripherals/behaviorCamera.hpp"
#include "../peripherals/arduinoCommunication.hpp"
#include "../common/calibration.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/cli.hpp"
#include "../common/behaviorRecording.hpp"
#include "../common/muscleRecording.hpp"
#include "../common/trackingControl.hpp"
#include "gui.hpp"

// Forward declarations
class QApplication;

// Program control functions
void initializeProgram();
bool quitProgram();

#endif // MAIN_HPP