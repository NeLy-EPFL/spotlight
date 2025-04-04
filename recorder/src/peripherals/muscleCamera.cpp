#include "muscleCamera.hpp"

MuscleCamera::MuscleCamera(double exposureTimeMicrosecs,
                           int imageWidth,
                           int imageHeight,
                           int xOffset,
                           int yOffset)
{
    spdlog::info("Setting up PCO (muscle) camera");
    camera_.defaultConfiguration();
    pco::Configuration config = camera_.getConfiguration();
    config.roi.x0 = xOffset + 1;
    config.roi.y0 = yOffset + 1;
    config.roi.x1 = xOffset + imageWidth;
    config.roi.y1 = yOffset + imageHeight;
    config.trigger_mode = TRIGGER_MODE_EXTERNALTRIGGER;
    config.acquire_mode = ACQUIRE_MODE_AUTO;
    config.delay_time_s = 0;
    config.noise_filter_mode = NOISE_FILTER_MODE_ON;
    camera_.setConfiguration(config);
    spdlog::info("PCO camera configuration set");

    camera_.setExposureTime(exposureTimeMicrosecs / 1000000.0);
    camera_.autoExposureOff();
    spdlog::info("PCO camera exposure time set to {} microseconds",
                 exposureTimeMicrosecs);
}

MuscleCamera::~MuscleCamera()
{
    stop();
}

void MuscleCamera::start(unsigned int bufferSize)
{
    spdlog::info("Starting PCO camera");
    camera_.record(bufferSize, pco::RecordMode::ring_buffer);
    cameraReadyFlag_.store(true);
    spdlog::info("PCO camera started");
}

void MuscleCamera::stop()
{
    camera_.stop();
}

FrameData MuscleCamera::waitForOneFrame()
{
    if (!firstFrameHasArrived_)
    {
        camera_.waitForFirstImage();
        firstFrameHasArrived_ = true;
    }
    else
    {
        camera_.waitForNewImage();
    }

    uint64_t receivedTime = getCurrentTimeMicroseconds();
    camera_.image(currentPCOImage_,
                  PCO_RECORDER_LATEST_IMAGE,
                  pco::DataFormat::Mono16);

    FrameData frameData;
    frameData.receivedTime = receivedTime;
    frameData.image = cv::Mat(imageHeight_,
                              imageWidth_,
                              CV_16UC1,
                              currentPCOImage_.raw_data().first);
    return frameData;
}

bool MuscleCamera::isReady() const
{
    return cameraReadyFlag_.load();
}

int roundToNearestValidMuscleCamWidth(int initialValue)
/**
 * @brief ROI width must be a multiple of 32 (see pco.panda 4.2/
 * pco.panda 4.2 bi/pco.panda 4.2 bi UV User Manual, Appendix A1.1.)
 */
{
    int remainder = initialValue % 32;
    return initialValue - remainder + (remainder < 16 ? 0 : 32);
}

int roundToNearestValidMuscleCamHeight(int initialValue)
/**
 * @brief ROI height must be a multiple of 8 (see pco.panda 4.2/
 * pco.panda 4.2 bi/pco.panda 4.2 bi UV User Manual, Appendix A1.1.)
 */
{
    int remainder = initialValue % 8;
    return initialValue - remainder + (remainder < 4 ? 0 : 8);
}