// #ifndef MUSCLE_RECORDING_HPP
// #define MUSCLE_RECORDING_HPP

// #include <iostream>
// #include <mutex>
// #include <queue>
// #include <condition_variable>
// #include <atomic>
// #include <fstream>

// #include <spdlog/spdlog.h>

// #include "peripherals/muscleCamera.hpp"
// #include "recorderConfig.hpp"

// struct MuscleRecordingState
// {
//     std::shared_ptr<MuscleCamera> muscleCamera = nullptr;
//     std::queue<GroupOfThreeFrames> muscleImageQueue;
//     std::mutex muscleImageQueueMutex;
//     std::condition_variable muscleImageQueueCondVar;
// };

// // Function declarations
// void muscleImageAcquierer();
// void muscleImageSaver();

// #endif // MUSCLE_RECORDING_HPP