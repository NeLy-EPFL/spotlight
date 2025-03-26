/**
 * Same as 
 * https://github.com/Excelitas-PCO/pco.cpp-samples/blob/main/src/SimpleExample/SimpleExample.cpp
 */

// SimpleExample.cpp
//
//#pragma once

#ifndef PCO_LINUX
#define PCO_LINUX
#endif

#include <stdio.h>
#include <string.h>
#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"

int main()
{
  try
  {
    int err = PCO_InitializeLib();
    if (err)
    {
      throw pco::CameraException(err);
    }
    pco::Camera cam;
    pco::Image img;
    int width, height;
    int image_count = 10;
    cam.defaultConfiguration();

    cam.setExposureTime(0.01);

    std::cout << "Enter Filepath where Images should go (default: .):" << std::endl;
    std::string path;
    std::getline(std::cin, path);
    if (path.empty())
      path = ".";

    cam.record(image_count, pco::RecordMode::sequence);

    for (int i = 0; i < image_count; i++)
    {
      if (cam.isColored()) { cam.image(img, i, pco::DataFormat::BGR8); }
      else { cam.image(img, i, pco::DataFormat::Mono8); }
      width = img.width();
      height = img.height();
      std::filesystem::path name = std::filesystem::absolute(path).append("img_" + std::to_string(i + 1) + ".tif");
      std::filesystem::path name_raw = std::filesystem::absolute(path).append("img_" + std::to_string(i + 1) + "_raw.tif");
      std::cout << "Image Count:" << i + 1 << " > " << name << std::endl;
      int err;
      if (cam.isColored())
      {
        err = PCO_RecorderSaveImage(img.raw_data().first, width, height, FILESAVE_IMAGE_BW_16, false, name_raw.string().c_str(), true, img.getMetaDataPtr());
        if (err)
        {
          throw pco::CameraException(err);
        }
        err = PCO_RecorderSaveImage(img.data().first, width, height, FILESAVE_IMAGE_BGR_8, true, name.string().c_str(), true, img.getMetaDataPtr());
        if (err)
        {
          throw pco::CameraException(err);
        }
      }
      else {
        err = PCO_RecorderSaveImage(img.raw_data().first, width, height, FILESAVE_IMAGE_BW_16, false, name_raw.string().c_str(), true, img.getMetaDataPtr());
        if (err)
        {
          throw pco::CameraException(err);
        }
        err = PCO_RecorderSaveImage(img.data().first, width, height, FILESAVE_IMAGE_BW_8, true, name.string().c_str(), true, img.getMetaDataPtr());
        if (err)
        {
          throw pco::CameraException(err);
        }
      }
    }

  }
  catch (pco::CameraException& err)
  {
    std::cout << "Error Code: " << err.error_code() << std::endl;
    std::cout << err.what() << std::endl;
  }
  PCO_CleanupLib();

  return 0;
}
