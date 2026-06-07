#pragma once

#include <cstdlib> // exit
#include <iostream>
#include <signal.h> // kill, SIGINT
#include <string>
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <unistd.h>    // fork, exec

#include <spdlog/spdlog.h>

#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"
#include "recorder/apps/shared_memory_utils.h"

// Derives the muscle camera's shutter-open window and the controller's
// muscle-trigger delay from the recording parameters, and validates that the
// requested muscle frame interval is long enough to fit the rolling shutter,
// light-on, and sensor readout times. See docs/data_acquisition.md.
class MuscleTriggerTiming {
  public:
    MuscleTriggerTiming(
        int behaviorCameraFPS, int syncRatio, int muscleLightOnTimeUs)
        : behaviorCameraFPS_(behaviorCameraFPS), syncRatio_(syncRatio),
          muscleLightOnTimeUs_(muscleLightOnTimeUs) {}

    // Populates the derived getters below. Returns false if the configuration
    // is invalid (the muscle frame interval is too short).
    bool computeParameters(
        int muscleImageHeight,
        double muscleCameraLineScanTimeUs,
        int muscleCameraReadoutTimeUs);

    int getMuscleShutterOpenTimeUs() const {
        return muscleShutterOpenTimeUs_;
    }
    int getMuscleCamTriggerDelayUs() const {
        return muscleCamTriggerDelayUs_;
    }

  private:
    // User input parameters
    int behaviorCameraFPS_;
    int syncRatio_;
    int muscleLightOnTimeUs_;

    // Derived parameters
    int muscleShutterOpenTimeUs_ = -1;
    int muscleCamTriggerDelayUs_ = -1;
};

class MuscleCamera {
  public:
    MuscleCamera(
        int imageWidth,
        int imageHeight,
        int xOffset,
        int yOffset,
        double rollingShutterLineTimeUs,
        double sensorReadoutTimeUs,
        const RecorderConfig &recorderConfig,
        std::string profileDir,
        spdlog::level::level_enum logLevel);
    ~MuscleCamera();
    FrameData waitForOneFrame();
    void setLightOnTime(unsigned int lightOnTimeMicrosecs);
    pid_t getCameraServerPID() const;
    int getNumLinesScanned() const;

  private:
    unsigned int x0_;
    unsigned int x1_;
    unsigned int y0_;
    unsigned int y1_;
    unsigned int imageWidth_;
    unsigned int imageHeight_;
    double rollingShutterLineTimeUs_;
    double sensorReadoutTimeUs_;
    pid_t pcoCameraServerPID_;
    uint8_t *frameDataPtr_;
    unsigned int *shutterOpenTimePtr_;
    PCOSharedMemory::FrameMetadata *frameMetadataPtr_;
    pthread_mutex_t *mutexPtr_;
    pthread_cond_t *condVarPtr_;
    const RecorderConfig &recorderConfig_;
    unsigned int lastFrameCount_;
    bool isReady_ = false;

    bool isROIValid();
};

int roundToNearestValidMuscleCamHorizontal(int value);
int roundToNearestValidMuscleCamVertical(int value);
