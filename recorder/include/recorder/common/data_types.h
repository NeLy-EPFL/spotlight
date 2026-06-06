#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>

struct FrameData {
    unsigned int frameId = -1;
    uint64_t acquisitionTime = 0; // as returned by frame grabber
    uint64_t receivedTime = 0;    // as returned by frame grabber
    cv::Mat image;
};

struct BehaviorCameraROI {
    unsigned int imageWidth;
    unsigned int imageHeight;
    unsigned int xOffset;
    unsigned int yOffset;
};

struct GroupOfThreeFrames {
    FrameData frame0;
    FrameData frame1;
    FrameData frame2;
    // Number of frames actually acquired (1, 2, or 3). A recording whose total
    // behavior-frame count is not a multiple of three ends on a partial group:
    // the unused trailing channels are saved black and not logged in the CSV.
    int numValidFrames = 3;
};

struct SerialPortInfo {
    std::string portName;
    std::string description;
    std::string manufacturer;
};

// Request type enum
enum MotionStageRequestType {
    GET_CURRENT_POSITION,
    SET_TARGET_POSITION,
    WAIT_UNTIL_IDLE,
    CHECK_IF_IDLE,
    START_HOMING
};

// Position structure
enum PositionType { ABSOLUTE, RELATIVE };

struct MotionStagePosition {
    double xPosMm;
    double yPosMm;
    PositionType positionType;
};

// Single request and response structure for all request types
struct MotionStageRequest {
    size_t clientIdHash;
    MotionStageRequestType requestType;
    MotionStagePosition position;
    float velocity;
};

struct MotionStageResponse {
    MotionStagePosition position = MotionStagePosition{
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN()};
    bool isIdle = false;
    bool setSuccess = false;
};

enum CalibrationScanDirection { ROW_BY_ROW, COLUMN_BY_COLUMN };

struct ProgramState {
    std::atomic<bool> toQuit = false;
    std::atomic<bool> isRecording = false;
};

struct ProgrammedStop {
    int numBehaviorFramesExpected = -1;
    int numMuscleFramesExpected = -1;
    std::atomic<bool> hasEndedFlagForGUI = false;
};

