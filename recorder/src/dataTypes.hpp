#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <opencv2/opencv.hpp>

struct FrameData
{
    long unsigned int acquisitionTime;
    long unsigned int receivedTime;
    cv::Mat *imagePtr;
    bool noMoreData = false;
};

struct SerialPortInfo
{
    std::string portName;
    std::string description;
    std::string manufacturer;
};

enum CameraAcquisitionMode {
    STREAM,
    RECORD
};

#endif // DATA_TYPES_HPP
