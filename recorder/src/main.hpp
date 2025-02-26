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
#include "peripherals/behaviorCamera.hpp"

// Forward declarations
class QApplication;

// Image acquisition
extern BehaviorCamera *behaviorCamera;
extern std::queue<GroupOfThreeFrames> behaviorImageQueue;
extern std::mutex behaviorImageQueueMutex;
extern std::condition_variable behaviorImageQueueCondVar;
extern std::atomic<bool> behaviorCameraReady;
extern std::queue<GroupOfThreeFrames> muscleImageQueue;
extern std::mutex muscleImageQueueMutex;
extern std::condition_variable muscleImageQueueCondVar;

// Motion control
extern std::atomic<bool> motionControlHandlerReady;

// Saving to disk
// Note: For simplicity, `saveDirectory` is not thread-safe. Realistically,
// the only time this variable is modified is when the user manually types or
// selects a path using the pop-up window. Moving the cursor to the record
// button is certainly slow enough that the variable must be ready by the
// time it's accessed when recording starts. This makes the saving threads to
// slightly faster (probably 100us or so per 3 frames?) because there's no
// mutex to acquire/release.
extern std::string saveDirectory;

// Streaming latest data (for live display and motion control)
extern FrameData latestFrameData;
extern std::mutex latestFrameMutex;

// Program lifetime and state
extern std::atomic<bool> toQuit;
extern std::atomic<bool> isRecording;
extern QApplication *application;

// Program control functions
void initializeProgram();
void quitProgram();

#endif // MAIN_HPP