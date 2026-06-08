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

// Derives the muscle camera's continuous-mode (auto-sequence) exposure from the
// recording parameters and validates that the requested muscle frame interval is
// long enough to fit the rolling shutter, light-on, and sensor readout times.
//
// In continuous mode the camera free-runs at 1/(nominalExposure + readout), so
// the nominal per-line exposure programmed into the camera is set so that one
// frame fills the desired muscle frame interval:
//   nominalExposure = muscleInterval - readout = rollingTime + commonTime
//   commonTime      = lightOn + bufferTime   (the SMA#4 common-time HIGH window)
// The blue excitation LED is pulsed for lightOn within the common time, which the
// trigger firmware locks to via the SMA#4 status line. See
// docs/data_acquisition.md.
class MuscleTriggerTiming {
  public:
    MuscleTriggerTiming(
        int behaviorCameraFPS, int syncRatio, int muscleLightOnTimeUs)
        : behaviorCameraFPS_(behaviorCameraFPS), syncRatio_(syncRatio),
          muscleLightOnTimeUs_(muscleLightOnTimeUs) {}

    // Populates the derived getters below. Returns false if the configuration
    // is invalid (the muscle frame interval is too short to fit the light-on
    // window inside the common time, i.e. the buffer time would be negative).
    bool computeParameters(
        int muscleImageHeight,
        double muscleCameraLineScanTimeUs,
        int muscleCameraReadoutTimeUs);

    // Nominal per-line exposure to program into the camera (us).
    int getNominalExposureUs() const {
        return nominalExposureUs_;
    }
    // Slack in the common-time window beyond the light-on time (us).
    int getBufferTimeUs() const {
        return bufferTimeUs_;
    }
    // Duration of the common time / SMA#4 HIGH window (us).
    int getCommonTimeUs() const {
        return commonTimeUs_;
    }

  private:
    // User input parameters
    int behaviorCameraFPS_;
    int syncRatio_;
    int muscleLightOnTimeUs_;

    // Derived parameters
    int nominalExposureUs_ = -1;
    int bufferTimeUs_ = -1;
    int commonTimeUs_ = -1;
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
    // Program the camera's nominal per-line exposure (us). In continuous
    // (auto-sequence) mode this also sets the free-run frame rate, since the
    // camera runs at 1/(nominalExposure + readout). Derive the value with
    // MuscleTriggerTiming::getNominalExposureUs().
    void setNominalExposureUs(unsigned int exposureUs);
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

    bool isROIValid();
};

int roundToNearestValidMuscleCamHorizontal(int value);
int roundToNearestValidMuscleCamVertical(int value);
