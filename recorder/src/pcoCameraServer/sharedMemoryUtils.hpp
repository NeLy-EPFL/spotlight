#ifndef SHARED_MEMORY_UTILS_HPP
#define SHARED_MEMORY_UTILS_HPP

#include <string>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>

#include <spdlog/spdlog.h>

namespace PCOSharedMemory
{
    struct FrameMetadata
    {
        unsigned int frameCount = 0;
        uint64_t acquisitionTime = 0;
    };

    void setupFrameData(const std::string &shmFrameDataName,
                        const size_t frameBufferSize,
                        uint8_t *&frameDataPtr);
    void setupExposureTime(const std::string &shmExposureTimeName,
                           unsigned int *&exposureTimePtr);
    void setupFrameMetadata(const std::string &shmExposureTimeName,
                            FrameMetadata *&exposureTimePtr);
    void setupMutex(const std::string &shmMutexName,
                    pthread_mutex_t *&mutexPtr);
    void setupConditionVariable(const std::string &shmCondVarName,
                                pthread_cond_t *&condVarPtr);
}

#endif // SHARED_MEMORY_UTILS_HPP