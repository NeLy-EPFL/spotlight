#include "sharedMemoryUtils.hpp"

namespace PCOSharedMemory
{

    void setupFrameData(const std::string &shmFrameDataName,
                        const size_t frameBufferSize,
                        uint8_t *&frameDataPtr)
    {
        int shmFileDescFrameData = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescFrameData == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescFrameData, frameBufferSize) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        frameDataPtr = (uint8_t *)mmap(nullptr,
                                       frameBufferSize,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED,
                                       shmFileDescFrameData,
                                       0);
        if (frameDataPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescFrameData);
    }

    void setupFrameCount(const std::string &shmFrameCountName,
                         unsigned int *&frameCountPtr)
    {
        int shmFileDescFrameCount = shm_open(
            shmFrameCountName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescFrameCount == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescFrameCount, sizeof(unsigned int)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        frameCountPtr = (unsigned int *)mmap(
            0,
            sizeof(unsigned int),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescFrameCount,
            0);
        if (frameCountPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescFrameCount);
    }

    void setupExposureTime(const std::string &shmExposureTimeName,
                           unsigned int *&exposureTimePtr)
    {
        int shmFileDescExposureTime = shm_open(
            shmExposureTimeName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescExposureTime == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        size_t exposureTimeSize = sizeof(unsigned int);
        if (ftruncate(shmFileDescExposureTime, exposureTimeSize) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        exposureTimePtr = (unsigned int *)mmap(
            0,
            exposureTimeSize,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescExposureTime,
            0);
        if (exposureTimePtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescExposureTime);
    }

    void setupMutex(const std::string &shmMutexName,
                    pthread_mutex_t *&mutex)
    {
        int shmFileDescMutex = shm_open(
            shmMutexName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDescMutex == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDescMutex, sizeof(pthread_mutex_t)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        mutex = (pthread_mutex_t *)mmap(
            0,
            sizeof(pthread_mutex_t),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDescMutex,
            0);
        if (mutex == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDescMutex);

        // First-time init for mutex
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(mutex, &attr);
    }

}