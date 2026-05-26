#ifndef MUSCLE_CAMERA_HPP
#define MUSCLE_CAMERA_HPP

#include <cstdlib> // exit
#include <iostream>
#include <signal.h> // kill, SIGINT
#include <string>
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <unistd.h>    // fork, exec

#include <spdlog/spdlog.h>

#include "../common/dataTypes.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "../pcoCameraServer/sharedMemoryUtils.hpp"

class DualRecordingConfig {
  public:
    DualRecordingConfig()
        : recordBoth_(false), behaviorCameraFPS_(0), syncRatio_(1), muscleLightOnTimeUs_(0) {}
    DualRecordingConfig(int behaviorCameraFPS, int syncRatio, int muscleLightOnTimeUs,
                        bool recordBoth = true)
        : recordBoth_(recordBoth), behaviorCameraFPS_(behaviorCameraFPS), syncRatio_(syncRatio),
          muscleLightOnTimeUs_(muscleLightOnTimeUs) {}

    bool isRecordingBoth() const { return recordBoth_; }
    int getBehaviorCameraFPS() const { return behaviorCameraFPS_; }
    int getSyncRatio() const { return syncRatio_; }
    int getMuscleLightOnTimeUs() const { return muscleLightOnTimeUs_; }
    int getMuscleCamTriggerDelayUs() const { return muscleCamTriggerDelayUs_; }
    void setRecordBoth(bool recordBoth) { recordBoth_ = recordBoth; }
    void setBehaviorCameraFPS(int fps) { behaviorCameraFPS_ = fps; }
    void setSyncRatio(int ratio) { syncRatio_ = ratio; }
    void setMuscleLightOnTimeUs(int lightOnTimeUs) { muscleLightOnTimeUs_ = lightOnTimeUs; }

    bool computeParameters(int muscleImageHeight, double muscleCameraLineScanTimeUs,
                           int muscleCameraReadoutTimeUs);
    void saveToFile(const std::string &yamlPath);

  private:
    // User input parameters
    bool recordBoth_;
    int behaviorCameraFPS_;
    int syncRatio_;
    int muscleLightOnTimeUs_;

    // Derived parameters
    bool hasBeenChecked_ = false;
    int muscleShutterOpenTimeUs_ = -1;
    int muscleCamTriggerDelayUs_ = -1;
};

class MuscleCamera {
  public:
    MuscleCamera(int imageWidth, int imageHeight, int xOffset, int yOffset,
                 double rollingShutterLineTimeUs, double sensorReadoutTimeUs,
                 const RecorderConfig &recorderConfig, std::string profileDir,
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

#endif // MUSCLE_CAMERA_HPP