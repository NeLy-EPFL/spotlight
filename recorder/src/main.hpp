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

#include "calibration.hpp"
#include "peripherals/behaviorCamera.hpp"
#include "peripherals/arduinoCommunication.hpp"
#include "recorderConfig.hpp"
#include "cli.hpp"
#include "behaviorRecording.hpp"
#include "muscleRecording.hpp"
#include "trackingControl.hpp"
#include "gui.hpp"

// Forward declarations
class QApplication;

// Program control functions
void initializeProgram();
bool quitProgram();

#endif // MAIN_HPP