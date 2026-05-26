#include "sharedMemoryUtils.hpp"

namespace PCOSharedMemory {

void setupFrameData(
    const std::string &shmFrameDataName,
    const size_t frameBufferSize,
    uint8_t *&frameDataPtr,
    bool createNew) {
    int shmFileDesc = -1;

    if (createNew) {

        shmFileDesc = shm_open(
            shmFrameDataName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1) {
            std::string errorMessage =
                "Failed to open shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, frameBufferSize) == -1) {
            std::string errorMessage =
                "Failed to set size of shared memory for frame data: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
    }

    else {
        shmFileDesc = shm_open(shmFrameDataName.c_str(), O_RDWR, 0666);
    }

    frameDataPtr = (uint8_t *)mmap(
        nullptr,
        frameBufferSize,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shmFileDesc,
        0);
    if (frameDataPtr == MAP_FAILED) {
        std::string errorMessage =
            "Failed to map shared memory for frame data: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    close(shmFileDesc);
}

void setupShutterOpenTime(
    const std::string &shmShutterOpenTimeName,
    unsigned int *&shutterOpenTimePtr,
    bool createNew) {
    int shmFileDesc = -1;
    size_t shutterOpenTimeSize = sizeof(unsigned int);

    if (createNew) {
        shmFileDesc = shm_open(
            shmShutterOpenTimeName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1) {
            std::string errorMessage =
                "Failed to open shared memory for shutter-open time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, shutterOpenTimeSize) == -1) {
            std::string errorMessage =
                "Failed to set size of shared memory for shutter-open "
                "time: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
    } else {
        shmFileDesc = shm_open(shmShutterOpenTimeName.c_str(), O_RDWR, 0666);
    }

    shutterOpenTimePtr = (unsigned int *)mmap(
        0,
        shutterOpenTimeSize,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shmFileDesc,
        0);
    if (shutterOpenTimePtr == MAP_FAILED) {
        std::string errorMessage =
            "Failed to map shared memory for shutter-open time: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    close(shmFileDesc);
}

void setupFrameMetadata(
    const std::string &shmFrameMetadataName,
    FrameMetadata *&frameMetadataPtr,
    bool createNew) {
    int shmFileDesc = -1;

    if (createNew) {
        shmFileDesc = shm_open(
            shmFrameMetadataName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1) {
            std::string errorMessage =
                "Failed to open shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(FrameMetadata)) == -1) {
            std::string errorMessage =
                "Failed to set size of shared memory for frame metadata: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
    } else {
        shmFileDesc = shm_open(shmFrameMetadataName.c_str(), O_RDWR, 0666);
    }

    frameMetadataPtr = (FrameMetadata *)mmap(
        0,
        sizeof(FrameMetadata),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shmFileDesc,
        0);
    if (frameMetadataPtr == MAP_FAILED) {
        std::string errorMessage =
            "Failed to map shared memory for frame metadata: " +
            std::string(strerror(errno));
        spdlog::critical(errorMessage);
        throw std::runtime_error(errorMessage);
    }
    close(shmFileDesc);
}

void setupMutex(
    const std::string &shmMutexName,
    pthread_mutex_t *&mutexPtr,
    bool createNew) {
    int shmFileDesc = -1;

    if (createNew) {
        shmFileDesc =
            shm_open(shmMutexName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1) {
            std::string errorMessage =
                "Failed to open shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(pthread_mutex_t)) == -1) {
            std::string errorMessage =
                "Failed to set size of shared memory for mutex: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
    } else {
        shmFileDesc = shm_open(shmMutexName.c_str(), O_RDWR, 0666);
    }

    mutexPtr = (pthread_mutex_t *)mmap(
        0,
        sizeof(pthread_mutex_t),
        PROT_READ | PROT_WRITE | O_TRUNC,
        MAP_SHARED,
        shmFileDesc,
        0);
    if (mutexPtr == MAP_FAILED) {
        std::string errorMessage = "Failed to map shared memory for mutex: " +
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

void setupConditionVariable(
    const std::string &shmCondVarName,
    pthread_cond_t *&condVarPtr,
    bool createNew) {
    int shmFileDesc = -1;

    if (createNew) {
        shmFileDesc =
            shm_open(shmCondVarName.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
        if (shmFileDesc == -1) {
            std::string errorMessage =
                "Failed to open shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
        if (ftruncate(shmFileDesc, sizeof(pthread_cond_t)) == -1) {
            std::string errorMessage =
                "Failed to set size of shared memory for frame count: " +
                std::string(strerror(errno));
            spdlog::critical(errorMessage);
            throw std::runtime_error(errorMessage);
        }
    } else {
        shmFileDesc = shm_open(shmCondVarName.c_str(), O_RDWR, 0666);
    }

    condVarPtr = (pthread_cond_t *)mmap(
        0,
        sizeof(pthread_cond_t),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shmFileDesc,
        0);
    if (condVarPtr == MAP_FAILED) {
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
    int err = pthread_cond_init(condVarPtr, &attr);
    if (err != 0) {
        std::string msg = "Failed to initialize condition variable: " +
                          std::string(strerror(err));
        spdlog::critical(msg);
        throw std::runtime_error(msg);
    }
}
} // namespace PCOSharedMemory