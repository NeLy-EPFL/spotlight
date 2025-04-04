/**
 * Same as
 * https://github.com/Excelitas-PCO/pco.cpp-samples/blob/main/src/SimpleExample/SimpleExample.cpp
 */

// SimpleExample.cpp
//
// #pragma once

#ifndef PCO_LINUX
#define PCO_LINUX
#endif

#include <stdio.h>
#include <string.h>

#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"
#include "opencv2/opencv.hpp"

const int fullFrameWidth = 2048;
const int fullFrameHeight = 2048;

int calculateOffset(int fullFrameSize, int roiSize)
{
    return (fullFrameSize - roiSize) / 2;
}

void setupPCOCamera(pco::Camera &camera,
                    double exposureTime,
                    int imageWidth,
                    int imageHeight)
{
    int xOffset = calculateOffset(fullFrameWidth, imageWidth);
    int yOffset = calculateOffset(fullFrameHeight, imageHeight);

    // Set configuration
    std::cout << "Setting up camera" << std::endl;
    std::cout << "Getting default configuration" << std::endl;
    camera.defaultConfiguration();
    pco::Configuration config = camera.getConfiguration();
    config.roi.x0 = xOffset + 1;
    config.roi.y0 = yOffset + 1;
    config.roi.x1 = xOffset + imageWidth;
    config.roi.y1 = yOffset + imageHeight;
    config.trigger_mode = TRIGGER_MODE_EXTERNALTRIGGER;
    config.acquire_mode = ACQUIRE_MODE_AUTO; // TODO: ?
    config.delay_time_s = 0;
    config.noise_filter_mode = NOISE_FILTER_MODE_ON; // TODO: ?
    std::cout << "Setting configuration" << std::endl;
    camera.setConfiguration(config);
    std::cout << "Configuration set" << std::endl;

    // Set exposure time
    std::cout << "Setting exposure time" << std::endl;
    camera.setExposureTime(exposureTime); // TODO: unit?
    camera.autoExposureOff();
    std::cout << "Exposure time set" << std::endl;
}

int main()
{
    int err = PCO_InitializeLib();
    if (err)
    {
        throw pco::CameraException(err);
    }
    pco::Camera camera;
    setupPCOCamera(camera, 0.01, 1024, 1024);

    // Create, configure, and start a new recorder instance
    int bufferSize = 10;
    std::cout << "Setting recording mode" << std::endl;
    camera.record(bufferSize, pco::RecordMode::ring_buffer);
    std::cout << "Recording mode set" << std::endl;

    pco::Image pcoImage;
    cv::Mat cvImage;
    cv::Mat displayImage;
    bool isFirstFrame = true;
    std::cout << "Entering frame grabbing loop" << std::endl;
    int frameId = 0;
    while (true)
    {
        // std::cout << "Waiting for image" << std::endl;
        if (isFirstFrame)
        {
            camera.waitForFirstImage();
            isFirstFrame = false;
        }
        else
        {
            camera.waitForNewImage();
        }
        // std::cout << "Image ready" << std::endl;
        camera.image(pcoImage,
                     PCO_RECORDER_LATEST_IMAGE,
                     pco::DataFormat::Mono16);
        // std::cout << "Image received" << std::endl;
        cvImage = cv::Mat(pcoImage.height(),
                          pcoImage.width(),
                          CV_16UC1,
                          pcoImage.raw_data().first);
        std::string filename = "output/pco_image_" +
                               cv::format("%05d", frameId++) + ".tif";
        cv::imwrite(filename, cvImage);
        // cv::normalize(cvImage, displayImage, 0, 65535, cv::NORM_MINMAX);
        displayImage = cvImage * 80;
        cv::imshow("PCO Image", displayImage);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    camera.stop();
}