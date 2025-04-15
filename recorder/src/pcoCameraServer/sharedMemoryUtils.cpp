#include "sharedMemoryUtils.hpp"

namespace PCOSharedMemory
{

    void setupFrameData(const std::string &shmFrameDataName,
                        const size_t frameBufferSize,
                        uint8_t *&frameDataPtr)
    {
        int shmFileDesc = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, frameBufferSize) == -1)
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
                                       shmFileDesc,
                                       0);
        if (frameDataPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDesc);
    }

    void setupExposureTime(const std::string &shmExposureTimeName,
                           unsigned int *&exposureTimePtr)
    {
        int shmFileDesc = shm_open(
            shmExposureTimeName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        size_t exposureTimeSize = sizeof(unsigned int);
        if (ftruncate(shmFileDesc, exposureTimeSize) == -1)
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
            shmFileDesc,
            0);
        if (exposureTimePtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for exposure time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDesc);
    }

    void setupFrameMetadata(const std::string &shmExposureTimeName,
                            FrameMetadata *&exposureTimePtr)
    {
        int shmFileDesc = shm_open(
            shmExposureTimeName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(FrameMetadata)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        exposureTimePtr = (FrameMetadata *)mmap(
            0,
            sizeof(FrameMetadata),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDesc,
            0);
        if (exposureTimePtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDesc);
    }

    void setupMutex(const std::string &shmMutexName,
                    pthread_mutex_t *&mutexPtr)
    {
        int shmFileDesc = shm_open(
            shmMutexName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(pthread_mutex_t)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        mutexPtr = (pthread_mutex_t *)mmap(
            0,
            sizeof(pthread_mutex_t),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDesc,
            0);
        if (mutexPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDesc);

        // First-time init for mutex
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(mutexPtr, &attr);
    }

    void setupConditionVariable(const std::string &shmCondVarName,
                                pthread_cond_t *&condVarPtr)
    {
        int shmFileDesc = shm_open(
            shmCondVarName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1)
        {
            std::string errorMessage =
                "Failed to open shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(unsigned int)) == -1)
        {
            std::string errorMessage =
                "Failed to set size of shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        condVarPtr = (pthread_cond_t *)mmap(
            0,
            sizeof(unsigned int),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            shmFileDesc,
            0);
        if (condVarPtr == MAP_FAILED)
        {
            std::string errorMessage =
                "Failed to map shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        close(shmFileDesc);

        // First-time init for condition variable
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_cond_init(condVarPtr, &attr);
    }
}