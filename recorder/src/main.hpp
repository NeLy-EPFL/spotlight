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

#include "global.hpp"
#include "constants.hpp"
#include "calibration.hpp"
#include "peripherals/behaviorCamera.hpp"
#include "peripherals/triggering.hpp"

// Forward declarations
class QApplication;

// Program control functions
void initializeProgram();
bool quitProgram();

#endif // MAIN_HPP