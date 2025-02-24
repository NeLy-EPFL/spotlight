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