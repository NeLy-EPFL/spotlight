#include "recorder/common/utils.h"

#include <algorithm>

uint64_t getCurrentTimeMicroseconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

cv::Mat makePseudoBGRImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames) {
    // frame0 is always valid (a group is only ever flushed with >= 1 frame).
    // For a partial final group the missing channels are filled with black.
    const cv::Mat &referenceImage = groupOfThreeFrames.frame0.image;
    cv::Mat blackImage =
        cv::Mat::zeros(referenceImage.size(), referenceImage.type());
    std::vector<cv::Mat> channels = {
        groupOfThreeFrames.frame0.image,
        groupOfThreeFrames.numValidFrames > 1 ? groupOfThreeFrames.frame1.image
                                              : blackImage,
        groupOfThreeFrames.numValidFrames > 2 ? groupOfThreeFrames.frame2.image
                                              : blackImage};
    cv::Mat pseudoBGRImage;
    cv::merge(channels, pseudoBGRImage);
    reorientBehaviorImage(pseudoBGRImage, pseudoBGRImage);
    return pseudoBGRImage;
}

void reorientBehaviorImage(const cv::Mat &sourceImage, cv::Mat &targetImage) {
    cv::rotate(sourceImage, targetImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::flip(targetImage, targetImage, 1); // dim 1 is horizontal)
}

void reorientMuscleImage(const cv::Mat &sourceImage, cv::Mat &targetImage) {
    cv::rotate(sourceImage, targetImage, cv::ROTATE_90_COUNTERCLOCKWISE);
}

std::string makeMetadataStringFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames) {
    std::string metadataString = "frame_id,acquired_time_us,received_time_us\n";
    const FrameData frames[3] = {
        groupOfThreeFrames.frame0,
        groupOfThreeFrames.frame1,
        groupOfThreeFrames.frame2};
    // Only log frames that were actually acquired: a partial final group leaves
    // its remaining (black) channels unlogged.
    for (int i = 0; i < groupOfThreeFrames.numValidFrames; ++i) {
        metadataString += fmt::format(
            "{},{},{}\n",
            frames[i].frameId,
            frames[i].acquisitionTime,
            frames[i].receivedTime);
    }
    return metadataString;
}

std::string getSerialPortName(
    const std::string &deviceDescription,
    const std::string &deviceManufacturer) {
    std::vector<SerialPortInfo> allSerialPortInfo;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts()) {
        std::string portName = port.portName().toStdString();
        std::string description = port.description().toStdString();
        std::string manufacturer = port.manufacturer().toStdString();
        if (description == deviceDescription &&
            manufacturer == deviceManufacturer) {
            spdlog::info(
                "Serial port found. "
                "Port name: '{}', description: '{}', manufacturer: '{}'",
                portName,
                description,
                manufacturer);
            return portName;
        }
        allSerialPortInfo.push_back({portName, description, manufacturer});
    }

    spdlog::critical(
        "Serial port not found. "
        "I'm looking for manufacturer '{}', description '{}'. "
        "Available ports are:",
        deviceManufacturer,
        deviceDescription);
    for (const SerialPortInfo &serialPortInfo : allSerialPortInfo) {
        spdlog::critical(
            "* Port name: '{}', description: '{}', manufacturer: '{}'",
            serialPortInfo.portName,
            serialPortInfo.description,
            serialPortInfo.manufacturer);
    }
    throw std::runtime_error("Serial port not found.");
    return "";
}

int calculateBehaviorCameraPreviewWidth(
    int behaviorCameraPreviewHeight,
    int motionStageXRange,
    int motionStageYRange) {
    return behaviorCameraPreviewHeight *
           (static_cast<float>(motionStageXRange) / motionStageYRange);
}

fs::path prepareOutputFolder(const fs::path &directory, bool clearFolder) {
    fs::path processedDir;

    // Expand ~ to home directory
    if (!directory.empty() && directory.string().front() == '~') {
        const char *homeDir = getenv("HOME");
        if (homeDir) {
            processedDir = fs::path(homeDir) / directory.string().substr(2);
            spdlog::info(
                "Expanded ~ in directory path '{}' to '{}'",
                directory.string(),
                processedDir.string());
        } else {
            spdlog::error(
                "Failed to expand ~ in directory path '{}' because $HOME is "
                "not defined. Set the $HOME environment variable or use "
                "absolute path.",
                directory.string());
        }
    } else {
        processedDir = directory;
    }
    fs::path absoluteDir = fs::absolute(processedDir);

    try {
        fs::create_directories(absoluteDir);
        spdlog::info(
            "Created directory '{}' (if it didn't already exist)",
            absoluteDir.string());

        if (clearFolder) {
            for (const auto &entry : fs::directory_iterator(absoluteDir)) {
                fs::remove_all(entry);
            }
            spdlog::info(
                "Cleared content of directory '{}'", absoluteDir.string());
        }
    } catch (const fs::filesystem_error &e) {
        spdlog::error(
            "Failed to create directory '{}' or clear its content: {}",
            absoluteDir.string(),
            e.what());
        throw;
    }

    return absoluteDir;
}

size_t getMyThreadIdHash() {
    std::thread::id myThreadId = std::this_thread::get_id();
    size_t myThreadIdHash = std::hash<std::thread::id>{}(myThreadId);
    return myThreadIdHash;
}

std::string expandPath(const std::string &path) {
    // Check if the path starts with "~/"
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        // Get the HOME environment variable
        const char *homeDir = std::getenv("HOME");

        // If HOME is available, replace "~/" with the home directory
        if (homeDir) {
            std::filesystem::path expandedPath =
                std::filesystem::path(homeDir) / path.substr(2);
            return expandedPath.string();
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

void convert16BitTo8Bit(
    const cv::Mat &sourceImage, cv::Mat &targetImage, int vmin, int vmax) {
    // Linearly map the [vmin, vmax] window of the 16-bit image onto the 8-bit
    // range [0, 255]: pixels at or below vmin become black, pixels at or above
    // vmax become white. convertTo saturates out-of-range results, giving the
    // clamping for free.
    double alpha = 255.0 / std::max(1, vmax - vmin);
    double beta = -vmin * alpha;
    sourceImage.convertTo(targetImage, CV_8U, alpha, beta);
}

SaveDirectory::SaveDirectory(const std::string &directory) {
    std::lock_guard<std::mutex> lock(mutex_);
    directory_ = expandPath(directory);
}

void SaveDirectory::setDirectory(const std::string &directory) {
    std::lock_guard<std::mutex> lock(mutex_);
    directory_ = expandPath(directory);
}

std::filesystem::path SaveDirectory::getDirectory() {
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

LatestFrame::LatestFrame() : latestFrameData_({0, 0, 0, cv::Mat()}) {}

FrameData LatestFrame::getLatestFrameData() {
    std::lock_guard<std::mutex> lock(latestFrameMutex_);
    return latestFrameData_;
}

void LatestFrame::setLatestFrameData(const FrameData &frameData) {
    std::lock_guard<std::mutex> lock(latestFrameMutex_);
    latestFrameData_ = frameData;
}