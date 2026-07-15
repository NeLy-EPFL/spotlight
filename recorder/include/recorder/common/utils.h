#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "recorder/common/data_types.h"

namespace fs = std::filesystem;

uint64_t get_current_time_microseconds();

void reorient_behavior_image(
    const cv::Mat &source_image, cv::Mat &target_image);
void reorient_muscle_image(const cv::Mat &source_image, cv::Mat &target_image);
cv::Mat make_pseudo_bgr_image_from_three_frames(
    const GroupOfThreeFrames &group_of_three_frames);
std::string make_metadata_string_from_three_frames(
    const GroupOfThreeFrames &group_of_three_frames);

std::string get_serial_port_name(
    const std::string &device_description,
    const std::string &device_manufacturer);

int calculate_behavior_camera_preview_width(
    int behavior_camera_preview_height,
    int motion_stage_x_range,
    int motion_stage_y_range);

// Linearly interpolate the tracking move acceleration (mm/s^2) by the fly's
// pixel radius from the image center: accel_min at the center, accel_max at
// radius >= min(cols, rows)/2 - max_accel_margin_px, clamped in between.
double compute_tracking_acceleration(
    double fly_col_px,
    double fly_row_px,
    int img_cols,
    int img_rows,
    double accel_min,
    double accel_max,
    double max_accel_margin_px);

fs::path prepare_output_folder(const fs::path &directory, bool clear_folder);

size_t get_my_thread_id_hash();

std::string expand_path(const std::string &path);

// Map the [vmin, vmax] intensity window of a 16-bit image onto the 8-bit
// display range [0, 255], clamping values outside the window to black / white.
void convert16_bit_to8_bit(
    const cv::Mat &source_image, cv::Mat &target_image, int vmin, int vmax);

class SaveDirectory {
  public:
    SaveDirectory(const std::string &directory);
    void set_directory(const std::string &directory);
    std::filesystem::path get_directory() const;
    void initialize();

  private:
    mutable std::mutex mutex_;
    fs::path directory_;
};

class LatestFrame {
  public:
    LatestFrame();
    FrameData get_latest_frame_data() const;
    void set_latest_frame_data(const FrameData &frame_data);

  private:
    FrameData latest_frame_data_;
    mutable std::mutex latest_frame_mutex_;
};
