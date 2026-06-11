#pragma once

#include <cassert>
#include <csignal>
#include <functional>
#include <iostream>
#include <string>
#include <tuple>

#include <EGrabber.h>
#include <FormatConverter.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "recorder/common/data_types.h"
#include "recorder/common/utils.h"

class BehaviorCamera {
  public:
    BehaviorCamera(
        unsigned int image_width,
        unsigned int image_height,
        unsigned int x_offset,
        unsigned int y_offset,
        const std::string &io_line);
    ~BehaviorCamera();
    void start(size_t buffer_size = 40);
    void stop();
    FrameData wait_for_one_frame();
    bool is_ready() const;

  private:
    Euresys::EGenTL gen_tl_;
    Euresys::EGrabberCameraInfo camera_;
    std::unique_ptr<Euresys::EGrabber<>> frame_grabber_ptr_;
    std::unique_ptr<Euresys::FormatConverter> format_converter_ptr_;
    int image_width_;
    int image_height_;
    int x_offset_;
    int y_offset_;
    std::string io_line_;
    int current_fps_;
    std::atomic<bool> camera_ready_flag_{false};

    // Apply the full GenICam configuration to the grabber and camera: the ROI
    // and external-trigger setup plus the base configuration ported from
    // etc/euresys_config.js. Called once by the constructor.
    void configure();

    template <typename Module>
    bool set_integer_and_check(const std::string &key, int value);

    template <typename Module>
    bool set_string_and_check(const std::string &key, const std::string &value);
};

int round_to_nearest_valid_behavior_cam_dimension(int value);

std::tuple<int, int> get_centered_offsets(
    int image_width,
    int image_height,
    int full_frame_width,
    int full_frame_height);
