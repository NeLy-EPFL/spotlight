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
      ioLine_(ioLine),
      currentFPS_(0)
{
    using Euresys::DeviceModule;
    using Euresys::InterfaceModule;
    using Euresys::RemoteModule;

    std::cout << "Running GenTL eGrabber discovery..." << std::endl;
    Euresys::EGrabberDiscovery egrabberDiscovery(genTL_);
    egrabberDiscovery.discover();
    std::cout << "  OK - GenTL eGrabber discovery completed" << std::endl;

    std::cout << "Configuring camera..." << std::endl;
    camera_ = egrabberDiscovery.cameras(0);
    frameGrabberPtr_ = std::make_unique<Euresys::EGrabber<>>(camera_);
    Euresys::EGrabberInfo frameGrabberInfo = camera_.grabbers[0];
    std::string interfaceID = frameGrabberInfo.interfaceID;
    std::string deviceID = frameGrabberInfo.deviceID;
    std::string deviceVendorName = frameGrabberInfo.deviceVendorName;
    std::string deviceModelName = frameGrabberInfo.deviceModelName;
    std::cout << "  OK - Camera configured" << std::endl;
    std::cout << "  Interface ID: " << interfaceID << std::endl;
    std::cout << "  Device ID: " << deviceID << std::endl;
    std::cout << "  Device vendor: " << deviceVendorName << std::endl;
    std::cout << "  Device model: " << deviceModelName << std::endl;

    std::cout << "Setting sensor ROI..." << std::endl;
    setIntegerAndCheck<RemoteModule>("Width", imageWidth);
    setIntegerAndCheck<RemoteModule>("Height", imageHeight);
    setIntegerAndCheck<RemoteModule>("OffsetX", xOffset);
    setIntegerAndCheck<RemoteModule>("OffsetY", yOffset);
    std::cout << "  OK - Successfully set sensor ROI" << std::endl;

    std::cout << "Configuring trigger-related settings..." << std::endl;
    // Hardware counterpart: feed a trigger signal (high = active, exposure
    // time controlled by trigger width) to the `ioLine` of the frame grabber
    // In our case, this is TTLIO12 on the external IO plug.
    std::cout << "  Setting " << ioLine << " line as Input..." << std::endl;
    setStringAndCheck<InterfaceModule>("LineSelector", ioLine);
    setStringAndCheck<InterfaceModule>("LineMode", "Input");

    std::cout << "  Setting LIN1 to specified ioLine ("
              << ioLine << ")..." << std::endl;
    setStringAndCheck<InterfaceModule>("LineInputToolSelector", "LIN1");
    setStringAndCheck<InterfaceModule>("LineInputToolSource", ioLine);

    std::cout << "  Setting CameraControlMethod to EXTERNAL..." << std::endl;
    setStringAndCheck<DeviceModule>("CameraControlMethod", "EXTERNAL");

    std::cout << "  Disabling trigger for AcquisitionStart/AcquisitionEnd..."
              << std::endl;
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionStart");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionEnd");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");

    std::cout << "  Enabling trigger for FrameStart using CXPin as source..."
              << std::endl;
    setStringAndCheck<RemoteModule>("TriggerSelector", "FrameStart");
    // The following line must be excluded because when CameraControlMethod,
    // TriggerMode must be "On" for FrameStart. This option is grayed out.
    // setStringAndCheck<RemoteModule>("TriggerMode", "On");
    // Let's just check its value instead
    assert(frameGrabberPtr_->getString<RemoteModule>("TriggerMode") == "On");
    setStringAndCheck<RemoteModule>("TriggerSource", "CXPin");
    std::cout << "  OK - Successfully configured all trigger-related settings"
              << std::endl;

    std::cout << "Using CXP6 for faster data transmission..." << std::endl;
    setStringAndCheck<RemoteModule>("LinkConfig", "CXP6_X4");
    std::cout << "  OK - Successfully set LinkConfig to CXP6_X4" << std::endl;

    formatConverterPtr_ = std::make_unique<Euresys::FormatConverter>(genTL_);
}

BehaviorCamera::~BehaviorCamera() {}

void BehaviorCamera::start(size_t bufferCount)
{
    frameGrabberPtr_->reallocBuffers(bufferCount);
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
    frameData.frameId = currentFrameId_++;
    frameData.acquisitionTime = acquisitionTime;
    frameData.receivedTime = receivedTime;
    frameData.imagePtr = new cv::Mat(
        imageHeight_, imageWidth_, CV_8UC1, dataPtr);
    return frameData;
}

template <typename Module>
bool BehaviorCamera::setIntegerAndCheck(
    const std::string key, int value)
{
    std::cout << "    Setting "
              << key << " to "
              << value << "..." << std::endl;
    frameGrabberPtr_->setInteger<Module>(key, value);
    int retrievedValue = frameGrabberPtr_->getInteger<Module>(key);
    if (retrievedValue != value)
    {
        std::cerr << "    ERROR - Failed to set "
                  << key << " to "
                  << value << ". Retrieved value "
                  << retrievedValue << "after setting" << std::endl;
        return false;
    }
    return true;
}

template <typename Module>
bool BehaviorCamera::setStringAndCheck(
    const std::string key, const std::string value)
{
    std::cout << "    Setting "
              << key << " to "
              << value << "..." << std::endl;
    frameGrabberPtr_->setString<Module>(key, value);
    std::string retrievedValue = frameGrabberPtr_->getString<Module>(key);
    if (retrievedValue != value)
    {
        std::cerr << "    ERROR - Failed to set "
                  << key << " to "
                  << value << ". Retrieved value "
                  << retrievedValue << "after setting" << std::endl;
        return false;
    }
    return true;
}

int roundToMultiplesOf64(int value)
{
    int remainder = value % 64;
    return value - remainder + (remainder < 32 ? 0 : 64);
}

std::tuple<int, int> getCenteredOffsets(
    int imageWidth, int imageHeight, int fullFrameWidth, int fullFrameHeight)
{
    int xOffset = roundToMultiplesOf64((fullFrameWidth - imageWidth) / 2);
    int yOffset = roundToMultiplesOf64((fullFrameHeight - imageHeight) / 2);
    return std::make_tuple(xOffset, yOffset);
}
