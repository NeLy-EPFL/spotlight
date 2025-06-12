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
#include "../common/recorderConfig.hpp"
#include "../common/dataTypes.hpp"
#include "../common/utils.hpp"

class DualRecordingConfig
{
public:
    DualRecordingConfig()
        : recordBoth_(false),
          behaviorCameraFPS_(0),
          syncRatio_(1),
          muscleLightOnTimeUs_(0) {}

    bool isRecordingBoth() const { return recordBoth_; }
    int getBehaviorCameraFPS() const { return behaviorCameraFPS_; }
    int getSyncRatio() const { return syncRatio_; }
    int getMuscleLightOnTimeUs() const { return muscleLightOnTimeUs_; }
    int getMuscleCamDelayAfterTriggerUs() const { return muscleCamDelayAfterTriggerUs_; }
    int getNumBehaviorToMuscleLeadingCycles() const { return numBehaviorToMuscleLeadingCycles_; }
    void setRecordBoth(bool recordBoth) { recordBoth_ = recordBoth; }
    void setBehaviorCameraFPS(int fps) { behaviorCameraFPS_ = fps; }
    void setSyncRatio(int ratio) { syncRatio_ = ratio; }
    void setMuscleLightOnTimeUs(int lightOnTimeUs) { muscleLightOnTimeUs_ = lightOnTimeUs; }

    bool computeParameters(int muscleImageHeight,
                           double muscleCameraLineScanTimeUs,
                           int muscleCameraReadoutTimeUs);
    void saveToFile(const std::string &yamlPath);

private:
    bool recordBoth_;
    int behaviorCameraFPS_;
    int syncRatio_;
    int muscleLightOnTimeUs_;
    int muscleShutterOpenTimeUs_ = 0;
    int hasBeenChecked_ = false;
    int muscleCamDelayAfterTriggerUs_;
    int numBehaviorToMuscleLeadingCycles_;
};

class MuscleCamera
{
public:
    MuscleCamera(int imageWidth,
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
    void setExposureTime(unsigned int lightOnTimeMicrosecs);
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