#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <opencv2/opencv.hpp>

struct FrameData
{
    long unsigned int acquisitionTime;
    long unsigned int receivedTime;
    cv::Mat *imagePtr;
};

#endif // DATA_TYPES_HPP
