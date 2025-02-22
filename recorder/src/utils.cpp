#include "utils.hpp"

uint64_t getCurrentTimeMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

std::string getSerialPortName(std::string deviceDescription,
                              std::string deviceManufacturer)
{
    std::vector<SerialPortInfo> allSerialPortInfo;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
    {
        std::string portName = port.portName().toStdString();
        std::string description = port.description().toStdString();
        std::string manufacturer = port.manufacturer().toStdString();
        if (description == deviceDescription &&
            manufacturer == deviceManufacturer)
        {
            spdlog::info(
                "Serial port found. "
                "Port name: '{}', description: '{}', manufacturer: '{}'",
                portName, description, manufacturer);
            return portName;
        }
        allSerialPortInfo.push_back({portName, description, manufacturer});
    }

    spdlog::error(
        "Arduino serial port not found. "
        "I'm looking for manufacturer '{}', description '{}'. "
        "Available ports are:",
        deviceManufacturer, deviceDescription);
    for (SerialPortInfo serialPortInfo : allSerialPortInfo)
    {
        spdlog::error(
            "* Port name: '{}', description: '{}', manufacturer: '{}'",
            serialPortInfo.portName,
            serialPortInfo.description,
            serialPortInfo.manufacturer);
    }
    return "";
}

cv::Mat makePseudoRGBImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames)
{
    FrameData frame0 = groupOfThreeFrames.frame0;
    std::vector<cv::Mat> channels = {
        *groupOfThreeFrames.frame0.imagePtr,
        *groupOfThreeFrames.frame1.imagePtr,
        *groupOfThreeFrames.frame2.imagePtr};
    cv::Mat pseudoRGBImage;
    cv::merge(channels, pseudoRGBImage);
    return pseudoRGBImage;
}

std::string makeMetadataStringFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames)
{
    std::string metadataString = "";
    for (const FrameData &frameData : {
             groupOfThreeFrames.frame0,
             groupOfThreeFrames.frame1,
             groupOfThreeFrames.frame2})
    {
        metadataString += fmt::format(
            "frameId {}, acquisitionTime {}, receivedTime {}\n",
            frameData.frameId,
            frameData.acquisitionTime,
            frameData.receivedTime);
    }
    return metadataString;
}

CameraAcquisitionConfig::CameraAcquisitionConfig(
    CameraAcquisitionMode mode, int fps, int exposureTimeMicroseconds)
    : mode(mode),
      fps(fps),
      exposureTimeMicroseconds(exposureTimeMicroseconds)
{
}

CameraAcquisitionConfig::CameraAcquisitionConfig(std::string commandString)
{
    std::vector<std::string> tokens;
    std::istringstream iss(commandString);
    for (std::string token; std::getline(iss, token, ' ');)
    {
        tokens.push_back(token);
    }

    if (tokens.size() != 3)
    {
        spdlog::error("Invalid command string: {}", commandString);
        throw std::runtime_error("Invalid command string");
    }

    mode = static_cast<CameraAcquisitionMode>(std::stoi(tokens[0]));
    fps = std::stoi(tokens[1]);
    exposureTimeMicroseconds = std::stoi(tokens[2]);
}

std::string CameraAcquisitionConfig::toCommandString() const
{
    return fmt::format("{} {} {}",
                       mode, fps, exposureTimeMicroseconds);
}