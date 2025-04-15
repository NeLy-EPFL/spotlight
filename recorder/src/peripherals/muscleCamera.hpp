#ifndef MUSCLE_CAMERA_HPP
#define MUSCLE_CAMERA_HPP

#include <iostream>
#include <unistd.h>    // fork, exec
#include <signal.h>    // kill, SIGINT
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <cstdlib>     // exit
#include <string>

#include <spdlog/spdlog.h>

#include "../pcoCameraServer/sharedMemoryUtils.hpp"
#include "../recorderConfig.hpp"
#include "../dataTypes.hpp"
#include "../utils.hpp"

class MuscleCamera
{
public:
    MuscleCamera(int imageWidth,
                 int imageHeight,
                 int xOffset,
                 int yOffset,
                 const RecorderConfig &recorderConfig,
                 std::string profileDir,
                 spdlog::level::level_enum logLevel);
    ~MuscleCamera();
    // void start();
    // void stop();
    FrameData waitForOneFrame();
    // bool isReady() const;
    void setExposureTime(unsigned int exposureTimeMicrosecs);

private:
    unsigned int imageWidth_;
    unsigned int imageHeight_;
    unsigned int xOffset_;
    unsigned int yOffset_;
    pid_t pcoCameraServerPID_;
    uint8_t *frameDataPtr_;
    unsigned int *exposureTimePtr_;
    PCOSharedMemory::FrameMetadata *frameMetadataPtr_;
    pthread_mutex_t *mutexPtr_;
    pthread_cond_t *condVarPtr_;
    const RecorderConfig &recorderConfig_;
    unsigned int lastFrameCount_;
    // std::atomic<bool> cameraReadyFlag_{false};
};

#endif // MUSCLE_CAMERA_HPP