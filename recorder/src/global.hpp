#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "dataTypes.hpp"

// Image acquisition
extern std::queue<GroupOfThreeFrames> behaviorImageQueue;
extern std::mutex behaviorImageQueueMutex;
extern std::condition_variable behaviorImageQueueCondVar;
extern std::atomic<bool> behaviorCameraReady;
extern std::queue<GroupOfThreeFrames> muscleImageQueue;
extern std::mutex muscleImageQueueMutex;
extern std::condition_variable muscleImageQueueCondVar;

// Motion control
// TODO: remove these if not needed
// extern std::mutex motionStageRequestMutex;
// extern std::condition_variable motionStageRequestCondVar;
// extern std::mutex motionStageResponseMutex;
// extern std::condition_variable motionStageResponseCondVar;
// extern std::atomic<bool> newRequestForMotionstage;
// extern std::atomic<bool> newPositionFromMotionStage;
// extern MotionStageRequest latestMotionStageRequest;
// extern MotionStagePosition latestMotionStagePosition;
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
extern std::shared_ptr<std::atomic<bool>> toQuit;
extern std::shared_ptr<std::atomic<bool>> isRecording;