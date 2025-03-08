#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "dataTypes.hpp"
#include "calibration.hpp"

// Some .hpp includes this file so we can't include them here.
// Make forward declarations instead.
class BehaviorCamera;
class ArduinoTriggerControllerInterface;

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
extern MotionStagePosition latestMotionStagePosition;
extern std::mutex latestMotionStagePositionMutex;
extern std::atomic<bool> isCalibrating;
extern std::atomic<bool> shouldOverrideTracking;
extern std::atomic<double> overrideXPosAbsolute;
extern std::atomic<double> overrideYPosAbsolute;

// Calibration
extern CalibrationParams behaviorCamCalibrationParams;

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

// Hardware trigger control
extern ArduinoTriggerControllerInterface *triggerController;

// Program lifetime and state
extern std::atomic<bool> toQuit;
extern std::atomic<bool> isRecording;

// IO
extern std::mutex isIOInitializing;