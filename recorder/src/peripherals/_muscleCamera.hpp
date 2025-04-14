// #ifndef PCO_LINUX
// #define PCO_LINUX
// #endif

// #ifndef MUSCLE_CAMERA_HPP
// #define MUSCLE_CAMERA_HPP

// #include <atomic>

// // #include "pco_linux_defs.h"
// #include "stdafx.h"

// // Definition of byte by PCO's headers conflicts with std::byte in C++17
// // and Qt6, so undef it before continuing
// #ifdef byte
// #undef byte
// #endif

// #include "camera.h"
// #include "cameraexception.h"
// #include "sc2_defs.h"

// #include <spdlog/spdlog.h>
// #include <opencv2/opencv.hpp>

// #include "../dataTypes.hpp"
// #include "../utils.hpp"

// class MuscleCamera
// {
// public:
//     MuscleCamera(unsigned int exposureTimeMicrosecs,
//                  int imageWidth,
//                  int imageHeight,
//                  int xOffset,
//                  int yOffset);
//     ~MuscleCamera();
//     void start(unsigned int bufferCount = 20);
//     void stop();
//     FrameData waitForOneFrame();
//     bool isReady() const;
//     void setExposureTime(unsigned int exposureTimeMicrosecs);

// private:
//     unsigned int imageWidth_;
//     unsigned int imageHeight_;
//     unsigned int xOffset_;
//     unsigned int yOffset_;
//     pco::Camera camera_;
//     bool firstFrameHasArrived_;
//     pco::Image currentPCOImage_;
//     std::atomic<bool> cameraReadyFlag_{false}; // atomic for thread safety
// };

// int roundToNearestValidMuscleCamWidth(int initialValue);
// int roundToNearestValidMuscleCamHeight(int initialValue);

// #endif // MUSCLE_CAMERA_HPP