#include <opencv2/opencv.hpp>

struct FrameData
{
    long unsigned int acquisitionTime;
    long unsigned int receivedTime;
    cv::Mat *imagePtr;
};