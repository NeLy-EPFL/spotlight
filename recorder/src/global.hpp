#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "dataTypes.hpp"

extern std::queue<FrameData> behaviorImageQueue;
extern std::mutex behaviorImageQueueMutex;
extern std::condition_variable behaviorImageQueueCondVar;
extern std::queue<FrameData> muscleImageQueue;
extern std::mutex muscleImageQueueMutex;
extern std::condition_variable muscleImageQueueCondVar;

extern std::shared_ptr<std::atomic<bool>> toQuit;
extern std::shared_ptr<std::atomic<bool>> isRecording;

extern FrameData latestFrameData;
extern std::mutex latestFrameMutex;