#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <opencv2/opencv.hpp>

struct FrameData
{
    unsigned int frameId = -1;
    uint64_t acquisitionTime = 0; // as returned by frame grabber
    uint64_t receivedTime = 0;    // as returned by frame grabber
    cv::Mat *imagePtr;
};

struct GroupOfThreeFrames
{
    FrameData frame0;
    FrameData frame1;
    FrameData frame2;
};

struct SerialPortInfo
{
    std::string portName;
    std::string description;
    std::string manufacturer;
};

enum MotionStageRequestType
{
    SET_TARGET_POSITION,
    GET_CURRENT_POSITION,
    HOME,
};

struct MotionStagePosition
{
    float xPosAbsoluteMm = std::numeric_limits<double>::quiet_NaN();
    float yPosAbsoluteMm = std::numeric_limits<double>::quiet_NaN();
};

struct MotionStageRequest
{
    MotionStageRequestType requestType;
    MotionStagePosition position;
};

#endif // DATA_TYPES_HPP
