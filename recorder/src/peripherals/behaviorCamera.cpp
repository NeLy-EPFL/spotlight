#include "behaviorCamera.hpp"

BehaviorCamera::BehaviorCamera(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    std::string ioLine)
    : imageWidth_(imageWidth),
      imageHeight_(imageHeight),
      xOffset_(xOffset),
      yOffset_(yOffset),
      ioLine_(ioLine)
{
    using Euresys::DeviceModule;
    using Euresys::InterfaceModule;
    using Euresys::RemoteModule;

    spdlog::info("Running GenTL eGrabber discovery...");
    Euresys::EGrabberDiscovery egrabberDiscovery(genTL_);
    egrabberDiscovery.discover();
    spdlog::info("GenTL eGrabber discovery completed");

    spdlog::info("Configuring camera...");
    camera_ = egrabberDiscovery.cameras(0);
    frameGrabberPtr_ = std::make_unique<Euresys::EGrabber<>>(camera_);
    Euresys::EGrabberInfo frameGrabberInfo = camera_.grabbers[0];
    std::string interfaceID = frameGrabberInfo.interfaceID;
    std::string deviceID = frameGrabberInfo.deviceID;
    std::string deviceVendorName = frameGrabberInfo.deviceVendorName;
    std::string deviceModelName = frameGrabberInfo.deviceModelName;
    spdlog::info("Camera configured - interface ID: {}, device ID: {}, "
                 "device vendor: {}, device model: {}",
                 interfaceID, deviceID, deviceVendorName, deviceModelName);

    spdlog::info("Setting sensor ROI - width: {}, height: {}, "
                 "xOffset: {}, yOffset: {}",
                 imageWidth, imageHeight, xOffset, yOffset);
    // Set offset to 0 first - if the new image size is larger than the current
    // one, setting the new image size directly may fail if newSize + currOffset
    // exceeds the current image size.
    setIntegerAndCheck<RemoteModule>("OffsetX", 0);
    setIntegerAndCheck<RemoteModule>("OffsetY", 0);
    setIntegerAndCheck<RemoteModule>("Width", imageWidth);
    setIntegerAndCheck<RemoteModule>("Height", imageHeight);
    setIntegerAndCheck<RemoteModule>("OffsetX", xOffset);
    setIntegerAndCheck<RemoteModule>("OffsetY", yOffset);
    spdlog::info("Sensor ROI set");

    spdlog::info("Configuring trigger-related settings...");
    // Hardware counterpart: feed a trigger signal (high = active, exposure
    // time controlled by trigger width) to the `ioLine` of the frame grabber
    // In our case, this is TTLIO12 on the external IO plug.
    spdlog::info("Setting {} line as Input...", ioLine);
    setStringAndCheck<InterfaceModule>("LineSelector", ioLine);
    setStringAndCheck<InterfaceModule>("LineMode", "Input");

    spdlog::info("Setting LIN1 to specified ioLine ({})...", ioLine);
    setStringAndCheck<InterfaceModule>("LineInputToolSelector", "LIN1");
    setStringAndCheck<InterfaceModule>("LineInputToolSource", ioLine);

    spdlog::info("Setting CameraControlMethod to EXTERNAL...");
    setStringAndCheck<DeviceModule>("CameraControlMethod", "EXTERNAL");

    spdlog::info("Disabling trigger for AcquisitionStart/AcquisitionEnd...");
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionStart");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionEnd");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");

    spdlog::info("Enabling trigger for FrameStart, using CXPin as source...");
    setStringAndCheck<RemoteModule>("TriggerSelector", "FrameStart");
    // The following line must be excluded because when CameraControlMethod,
    // TriggerMode must be "On" for FrameStart. This option is grayed out.
    // setStringAndCheck<RemoteModule>("TriggerMode", "On");
    // Let's just check its value instead
    assert(frameGrabberPtr_->getString<RemoteModule>("TriggerMode") == "On");
    setStringAndCheck<RemoteModule>("TriggerSource", "CXPin");
    spdlog::info("Trigger-related settings configured");

    spdlog::info("Setting LinkConfig to CXP6_X4...");
    setStringAndCheck<RemoteModule>("LinkConfig", "CXP6_X4");
    spdlog::info("LinkConfig set");

    formatConverterPtr_ = std::make_unique<Euresys::FormatConverter>(genTL_);

    cameraReadyFlag_.store(true);
}

BehaviorCamera::~BehaviorCamera()
{
    spdlog::debug("Behavior camera destructor called");
    cameraReadyFlag_.store(false);
}

void BehaviorCamera::start(size_t bufferSize)
{
    frameGrabberPtr_->reallocBuffers(bufferSize);
    frameGrabberPtr_->start();
}

void BehaviorCamera::stop()
{
    frameGrabberPtr_->stop();
}

FrameData BehaviorCamera::waitForOneFrame()
{
    // Getting the buffer is the main blocking call
    Euresys::ScopedBuffer buffer(*frameGrabberPtr_);

    // Get image data and metadata
    uint64_t receivedTime = getCurrentTimeMicroseconds();
    uint8_t *dataPtr = buffer.getInfo<uint8_t *>(
        Euresys::gc::BUFFER_INFO_BASE);
    uint64_t grabberTimestamp = buffer.getInfo<uint64_t>(
        Euresys::gc::BUFFER_INFO_TIMESTAMP_NS);
    uint64_t acquisitionTime = grabberTimestamp / 1000;

    // Make FrameData object
    FrameData frameData;
    frameData.acquisitionTime = acquisitionTime;
    frameData.receivedTime = receivedTime;
    frameData.image = cv::Mat(imageHeight_, imageWidth_, CV_8UC1, dataPtr);
    return frameData;
}

bool BehaviorCamera::isReady() const
{
    return cameraReadyFlag_.load();
}

template <typename Module>
bool BehaviorCamera::setIntegerAndCheck(
    const std::string key, int value)
{
    spdlog::info("Setting GenICam integer parameter {} to {}", key, value);
    frameGrabberPtr_->setInteger<Module>(key, value);
    int retrievedValue = frameGrabberPtr_->getInteger<Module>(key);
    if (retrievedValue != value)
    {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

template <typename Module>
bool BehaviorCamera::setStringAndCheck(
    const std::string key, const std::string value)
{
    spdlog::info("Setting GenICam string parameter {} to {}", key, value);
    frameGrabberPtr_->setString<Module>(key, value);
    std::string retrievedValue = frameGrabberPtr_->getString<Module>(key);
    if (retrievedValue != value)
    {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

int roundToNearestValidBehaviorCamDimension(int value)
{
    int remainder = value % 64;
    return value - remainder + (remainder < 32 ? 0 : 64);
}

std::tuple<int, int> getCenteredOffsets(
    int imageWidth, int imageHeight, int fullFrameWidth, int fullFrameHeight)
{
    int xOffset = roundToNearestValidBehaviorCamDimension(
        (fullFrameWidth - imageWidth) / 2);
    int yOffset = roundToNearestValidBehaviorCamDimension(
        (fullFrameHeight - imageHeight) / 2);
    return std::make_tuple(xOffset, yOffset);
}
