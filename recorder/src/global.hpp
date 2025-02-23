#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "dataTypes.hpp"

extern std::queue<GroupOfThreeFrames> behaviorImageQueue;
extern std::mutex behaviorImageQueueMutex;
extern std::condition_variable behaviorImageQueueCondVar;
extern std::queue<GroupOfThreeFrames> muscleImageQueue;
extern std::mutex muscleImageQueueMutex;
extern std::condition_variable muscleImageQueueCondVar;

extern std::shared_ptr<std::atomic<bool>> toQuit;
extern std::shared_ptr<std::atomic<bool>> isRecording;

// Note: For simplicity, `saveDirectory` is not thread-safe. Realistically,
// the only time this variable is modified is when the user manually types or
// selects a path using the pop-up window. Moving the cursor to the record
// button is certainly slow enough that the variable must be ready by the
// time it's accessed when recording starts. This makes the saving threads to
// slightly faster (probably 100us or so per 3 frames?) because there's no
// mutex to acquire/release.
extern std::string saveDirectory;

extern FrameData latestFrameData;
extern std::mutex latestFrameMutex;