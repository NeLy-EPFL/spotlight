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
    void setupFrameData(const std::string &shmFrameDataName,
                        const size_t frameBufferSize,
                        uint8_t *&frameDataPtr);
    void setupFrameCount(const std::string &shmFrameCountName,
                         unsigned int *&frameCountPtr);
    void setupExposureTime(const std::string &shmExposureTimeName,
                           unsigned int *&exposureTimePtr);
    void setupMutex(const std::string &shmMutexName,
                    pthread_mutex_t *&mutex);
}

#endif // SHARED_MEMORY_UTILS_HPP