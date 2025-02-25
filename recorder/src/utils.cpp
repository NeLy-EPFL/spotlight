#include "utils.hpp"

uint64_t getCurrentTimeMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
}

cv::Mat makePseudoRGBImageFromThreeFrames(
    const GroupOfThreeFrames &groupOfThreeFrames)
{
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
    std::string metadataString = "frameId,acquisitionTimeUs,receivedTimeUs\n";
    for (const FrameData &frameData : {
             groupOfThreeFrames.frame0,
             groupOfThreeFrames.frame1,
             groupOfThreeFrames.frame2})
    {
        metadataString += fmt::format(
            "{},{},{}\n",
            frameData.frameId,
            frameData.acquisitionTime,
            frameData.receivedTime);
    }
    return metadataString;
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
        "Serial port not found. "
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

std::tuple<int, int> calculateMaxMotionStageRequestHandlingTime()
{
    int getPositionWeight = 1;
    int setPositionWeight = 2; // Set position is more time-consuming

    int totalWeightedNumberOfOps =
        MOTION_STAGE_LOGGING_FREQUENCY_HZ * getPositionWeight +
        FLY_TRACKING_UPDATE_FREQUENCY_HZ *
            (setPositionWeight + getPositionWeight) + // this needs to do both
        GUI_MOTION_STAGE_PREVIEW_FREQUENCY_HZ * getPositionWeight;

    int numMicrosecsAllowedPerUnitOp = 1000000 / totalWeightedNumberOfOps;

    return std::make_tuple(
        numMicrosecsAllowedPerUnitOp * getPositionWeight,
        numMicrosecsAllowedPerUnitOp * setPositionWeight);
}