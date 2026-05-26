#ifndef SHARED_MEMORY_UTILS_HPP
#define SHARED_MEMORY_UTILS_HPP

#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

namespace PCOSharedMemory {
struct FrameMetadata {
    unsigned int frameCount = 0;
    uint64_t acquisitionTime = 0;
};

void setupFrameData(
    const std::string &shmFrameDataName,
    const size_t frameBufferSize,
    uint8_t *&frameDataPtr,
    bool createNew);
void setupShutterOpenTime(
    const std::string &shmShutterOpenTimeName,
    unsigned int *&shutterOpenTimePtr,
    bool createNew);
void setupFrameMetadata(
    const std::string &shmFrameMetadataName,
    FrameMetadata *&frameMetadataPtr,
    bool createNew);
void setupMutex(
    const std::string &shmMutexName,
    pthread_mutex_t *&mutexPtr,
    bool createNew);
void setupConditionVariable(
    const std::string &shmCondVarName,
    pthread_cond_t *&condVarPtr,
    bool createNew);
} // namespace PCOSharedMemory

#endif // SHARED_MEMORY_UTILS_HPP