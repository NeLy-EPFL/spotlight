#include "recorder/common/utils.h"

#include <algorithm>
#include <cmath>

uint64_t get_current_time_microseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

cv::Mat make_pseudo_bgr_image_from_three_frames(
    const GroupOfThreeFrames &group_of_three_frames) {
    // frame0 is always valid (a group is only ever flushed with >= 1 frame).
    // For a partial final group the missing channels are filled with black.
    const cv::Mat &reference_image = group_of_three_frames.frame0.image;
    cv::Mat black_image =
        cv::Mat::zeros(reference_image.size(), reference_image.type());
    std::vector<cv::Mat> channels = {
        group_of_three_frames.frame0.image,
        group_of_three_frames.num_valid_frames > 1
            ? group_of_three_frames.frame1.image
            : black_image,
        group_of_three_frames.num_valid_frames > 2
            ? group_of_three_frames.frame2.image
            : black_image};
    cv::Mat pseudo_bgr_image;
    cv::merge(channels, pseudo_bgr_image);
    reorient_behavior_image(pseudo_bgr_image, pseudo_bgr_image);
    return pseudo_bgr_image;
}

void reorient_behavior_image(
    const cv::Mat &source_image, cv::Mat &target_image) {
    cv::rotate(source_image, target_image, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::flip(target_image, target_image, 1); // dim 1 is horizontal)
}

void reorient_muscle_image(const cv::Mat &source_image, cv::Mat &target_image) {
    cv::rotate(source_image, target_image, cv::ROTATE_90_COUNTERCLOCKWISE);
}

std::string make_metadata_string_from_three_frames(
    const GroupOfThreeFrames &group_of_three_frames) {
    std::string metadata_string =
        "frame_id,acquired_time_us,received_time_us\n";
    const FrameData frames[3] = {
        group_of_three_frames.frame0,
        group_of_three_frames.frame1,
        group_of_three_frames.frame2};
    // Only log frames that were actually acquired: a partial final group leaves
    // its remaining (black) channels unlogged.
    for (int i = 0; i < group_of_three_frames.num_valid_frames; ++i) {
        metadata_string += fmt::format(
            "{},{},{}\n",
            frames[i].frame_id,
            frames[i].acquisition_time,
            frames[i].received_time);
    }
    return metadata_string;
}

std::string get_serial_port_name(
    const std::string &device_description,
    const std::string &device_manufacturer) {
    std::vector<SerialPortInfo> all_serial_port_info;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts()) {
        std::string port_name = port.portName().toStdString();
        std::string description = port.description().toStdString();
        std::string manufacturer = port.manufacturer().toStdString();
        if (description == device_description &&
            manufacturer == device_manufacturer) {
            spdlog::info(
                "Serial port found. "
                "Port name: '{}', description: '{}', manufacturer: '{}'",
                port_name,
                description,
                manufacturer);
            return port_name;
        }
        all_serial_port_info.push_back({port_name, description, manufacturer});
    }

    spdlog::critical(
        "Serial port not found. "
        "I'm looking for manufacturer '{}', description '{}'. "
        "Available ports are:",
        device_manufacturer,
        device_description);
    for (const SerialPortInfo &serial_port_info : all_serial_port_info) {
        spdlog::critical(
            "* Port name: '{}', description: '{}', manufacturer: '{}'",
            serial_port_info.port_name,
            serial_port_info.description,
            serial_port_info.manufacturer);
    }
    throw std::runtime_error("Serial port not found.");
    return "";
}

int calculate_behavior_camera_preview_width(
    int behavior_camera_preview_height,
    int motion_stage_x_range,
    int motion_stage_y_range) {
    return behavior_camera_preview_height *
           (static_cast<float>(motion_stage_x_range) / motion_stage_y_range);
}

double compute_tracking_acceleration(
    double fly_col_px,
    double fly_row_px,
    int img_cols,
    int img_rows,
    double accel_min,
    double accel_max,
    double max_accel_margin_px) {
    double dx = fly_col_px - img_cols / 2.0;
    double dy = fly_row_px - img_rows / 2.0;
    double radius_px = std::sqrt(dx * dx + dy * dy);
    double r_max = std::min(img_cols, img_rows) / 2.0 - max_accel_margin_px;
    double frac = (r_max > 0) ? std::clamp(radius_px / r_max, 0.0, 1.0) : 1.0;
    return accel_min + frac * (accel_max - accel_min);
}

fs::path prepare_output_folder(const fs::path &directory, bool clear_folder) {
    fs::path processed_dir;

    // Expand ~ to home directory
    if (!directory.empty() && directory.string().front() == '~') {
        const char *home_dir = getenv("HOME");
        if (home_dir) {
            processed_dir = fs::path(home_dir) / directory.string().substr(2);
            spdlog::info(
                "Expanded ~ in directory path '{}' to '{}'",
                directory.string(),
                processed_dir.string());
        } else {
            spdlog::error(
                "Failed to expand ~ in directory path '{}' because $HOME is "
                "not defined. Set the $HOME environment variable or use "
                "absolute path.",
                directory.string());
        }
    } else {
        processed_dir = directory;
    }
    fs::path absolute_dir = fs::absolute(processed_dir);

    try {
        fs::create_directories(absolute_dir);
        spdlog::info(
            "Created directory '{}' (if it didn't already exist)",
            absolute_dir.string());

        if (clear_folder) {
            for (const auto &entry : fs::directory_iterator(absolute_dir)) {
                fs::remove_all(entry);
            }
            spdlog::info(
                "Cleared content of directory '{}'", absolute_dir.string());
        }
    } catch (const fs::filesystem_error &e) {
        spdlog::error(
            "Failed to create directory '{}' or clear its content: {}",
            absolute_dir.string(),
            e.what());
        throw;
    }

    return absolute_dir;
}

size_t get_my_thread_id_hash() {
    std::thread::id my_thread_id = std::this_thread::get_id();
    size_t my_thread_id_hash = std::hash<std::thread::id>{}(my_thread_id);
    return my_thread_id_hash;
}

std::string expand_path(const std::string &path) {
    // Check if the path starts with "~/"
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        // Get the HOME environment variable
        const char *home_dir = std::getenv("HOME");

        // If HOME is available, replace "~/" with the home directory
        if (home_dir) {
            std::filesystem::path expanded_path =
                std::filesystem::path(home_dir) / path.substr(2);
            return expanded_path.string();
        } else {
            spdlog::error(
                "Failed to expand ~ in directory path '{}' because $HOME is "
                "not defined. Set the $HOME environment variable or use "
                "absolute path.",
                path.c_str());
        }
    }

    // Return the original path if it doesn't start with "~/"
    return path;
}

void convert16_bit_to8_bit(
    const cv::Mat &source_image, cv::Mat &target_image, int vmin, int vmax) {
    // Linearly map the [vmin, vmax] window of the 16-bit image onto the 8-bit
    // range [0, 255]: pixels at or below vmin become black, pixels at or above
    // vmax become white. convertTo saturates out-of-range results, giving the
    // clamping for free.
    double alpha = 255.0 / std::max(1, vmax - vmin);
    double beta = -vmin * alpha;
    source_image.convertTo(target_image, CV_8U, alpha, beta);
}

SaveDirectory::SaveDirectory(const std::string &directory) {
    std::lock_guard<std::mutex> lock(mutex_);
    directory_ = expand_path(directory);
}

void SaveDirectory::set_directory(const std::string &directory) {
    std::lock_guard<std::mutex> lock(mutex_);
    directory_ = expand_path(directory);
}

std::filesystem::path SaveDirectory::get_directory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return directory_;
}

void SaveDirectory::initialize() {
    try {
        fs::create_directories(directory_ / "behavior_images");
        fs::create_directories(directory_ / "muscle_images");
        fs::create_directories(directory_ / "stage_position");
        fs::create_directories(directory_ / "metadata");
    } catch (const fs::filesystem_error &e) {
        spdlog::error(
            "Failed to create directories in '{}': {}",
            directory_.string(),
            e.what());
        throw;
    }
}

LatestFrame::LatestFrame() : latest_frame_data_({0, 0, 0, 0, cv::Mat()}) {}

FrameData LatestFrame::get_latest_frame_data() const {
    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    return latest_frame_data_;
}

void LatestFrame::set_latest_frame_data(const FrameData &frame_data) {
    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    latest_frame_data_ = frame_data;
}