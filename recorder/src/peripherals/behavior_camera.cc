#include "recorder/peripherals/behavior_camera.h"

BehaviorCamera::BehaviorCamera(
    unsigned int imageWidth,
    unsigned int imageHeight,
    unsigned int xOffset,
    unsigned int yOffset,
    std::string ioLine)
    : imageWidth_(imageWidth), imageHeight_(imageHeight), xOffset_(xOffset),
      yOffset_(yOffset), ioLine_(ioLine) {
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
    spdlog::info(
        "Camera configured - interface ID: {}, device ID: {}, "
        "device vendor: {}, device model: {}",
        interfaceID,
        deviceID,
        deviceVendorName,
        deviceModelName);

    configure();

    formatConverterPtr_ = std::make_unique<Euresys::FormatConverter>(genTL_);

    cameraReadyFlag_.store(true);
}

void BehaviorCamera::configure() {
    using Euresys::DeviceModule;
    using Euresys::InterfaceModule;
    using Euresys::RemoteModule;

    // =======================================================================
    // Base grabber/camera setup, ported one-to-one from etc/euresys_config.js.
    // That script used to be run by hand (via eGrabber Studio) before launching
    // any recorder program; applying it here removes that manual step.
    //
    // A few of these settings are deliberately overridden further down by the
    // ROI and external-trigger configuration (CameraControlMethod is set to
    // EXTERNAL, the TTLIO12 line is turned into a trigger input, and the sensor
    // ROI comes from the recorder config) -- exactly as they were overridden
    // when the script was run by hand and the program reconfigured the grabber
    // afterwards. They are kept here so this method alone leaves the grabber
    // fully configured, and the original ordering is preserved (in particular
    // the ExposureMode Off -> TriggerWidth dance below) so that calling
    // configure() again is idempotent.
    // =======================================================================
    spdlog::info("Applying base grabber configuration (euresys_config.js)...");

    // Negotiate Power-over-CoaXPress so the camera is powered over the link.
    frameGrabberPtr_->execute<InterfaceModule>("CxpPoCxpAuto");

    // Camera trigger + exposure. ExposureMode has to leave TriggerWidth before
    // the FrameStart trigger can be (re)configured and then be restored to
    // TriggerWidth; the guard is what makes a repeat call idempotent.
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionStart");
    setStringAndCheck<RemoteModule>("TriggerMode", "On");
    setStringAndCheck<RemoteModule>("TriggerSource", "CXPin");
    setStringAndCheck<RemoteModule>("TriggerSelector", "FrameStart");
    if (frameGrabberPtr_->getString<RemoteModule>("ExposureMode") ==
        "TriggerWidth") {
        setStringAndCheck<RemoteModule>("ExposureMode", "Off");
    }
    setStringAndCheck<RemoteModule>("TriggerMode", "On");
    setStringAndCheck<RemoteModule>("TriggerSource", "CXPin");
    setStringAndCheck<RemoteModule>("ExposureMode", "TriggerWidth");
    setStringAndCheck<RemoteModule>("LinkConfig", "CXP6_X4");

    // Camera control method. "RG" is overridden to "EXTERNAL" below;
    // CycleTriggerSource is left as set here.
    setStringAndCheck<DeviceModule>("CameraControlMethod", "RG");
    setStringAndCheck<DeviceModule>("CycleTriggerSource", "Immediate");

    // Strobe output lines. TTLIO12 is overridden to a trigger input below (it
    // carries the external frame trigger from the microcontroller).
    setStringAndCheck<InterfaceModule>("LineSelector", "TTLIO11");
    setStringAndCheck<InterfaceModule>("LineSource", "Device0Strobe");
    setStringAndCheck<InterfaceModule>("LineMode", "Output");
    setStringAndCheck<InterfaceModule>("LineSelector", "TTLIO12");
    setStringAndCheck<InterfaceModule>("LineSource", "Device0Strobe");
    setStringAndCheck<InterfaceModule>("LineMode", "Output");
    // Numeric features: set without the strict string read-back check, since
    // they read back in a different textual form (e.g. "2" -> "2.000000").
    spdlog::info("Setting LineSourceDivisionFactor to 4 and Gain to 2");
    frameGrabberPtr_->setString<InterfaceModule>(
        "LineSourceDivisionFactor", "4");
    frameGrabberPtr_->setString<RemoteModule>("Gain", "2");

    // =======================================================================
    // Sensor ROI from the recorder config (supersedes the script's hard-coded
    // ROI). Set the offsets to 0 first - if the new image size is larger than
    // the current one, setting the new size directly may fail when
    // newSize + currentOffset exceeds the sensor bounds.
    // =======================================================================
    spdlog::info("Setting sensor OffsetX and OffsetY to 0, 0");
    setIntegerAndCheck<RemoteModule>("OffsetX", 0);
    setIntegerAndCheck<RemoteModule>("OffsetY", 0);

    spdlog::info(
        "Setting sensor ROI - width: {}, height: {}, "
        "xOffset: {}, yOffset: {}",
        imageWidth_,
        imageHeight_,
        xOffset_,
        yOffset_);
    setIntegerAndCheck<RemoteModule>("Width", imageWidth_);
    setIntegerAndCheck<RemoteModule>("Height", imageHeight_);
    setIntegerAndCheck<RemoteModule>("OffsetX", xOffset_);
    setIntegerAndCheck<RemoteModule>("OffsetY", yOffset_);
    spdlog::info("Sensor ROI set");

    // =======================================================================
    // External-trigger configuration. The microcontroller feeds a trigger
    // signal (high = active, exposure time controlled by trigger width) into
    // `ioLine_` (TTLIO12) of the frame grabber.
    // =======================================================================
    spdlog::info("Configuring trigger-related settings...");
    spdlog::info("Setting {} line as Input...", ioLine_);
    setStringAndCheck<InterfaceModule>("LineSelector", ioLine_);
    setStringAndCheck<InterfaceModule>("LineMode", "Input");

    spdlog::info("Setting LIN1 to specified ioLine ({})...", ioLine_);
    setStringAndCheck<InterfaceModule>("LineInputToolSelector", "LIN1");
    setStringAndCheck<InterfaceModule>("LineInputToolSource", ioLine_);

    spdlog::info("Setting CameraControlMethod to EXTERNAL...");
    setStringAndCheck<DeviceModule>("CameraControlMethod", "EXTERNAL");

    spdlog::info("Disabling trigger for AcquisitionStart/AcquisitionEnd...");
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionStart");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");
    setStringAndCheck<RemoteModule>("TriggerSelector", "AcquisitionEnd");
    setStringAndCheck<RemoteModule>("TriggerMode", "Off");

    spdlog::info("Enabling trigger for FrameStart, using CXPin as source...");
    setStringAndCheck<RemoteModule>("TriggerSelector", "FrameStart");
    // TriggerMode must be "On" for FrameStart when CameraControlMethod is
    // EXTERNAL (the option is grayed out / forced on), so it is checked rather
    // than set here.
    assert(frameGrabberPtr_->getString<RemoteModule>("TriggerMode") == "On");
    setStringAndCheck<RemoteModule>("TriggerSource", "CXPin");
    spdlog::info("Trigger-related settings configured");

    spdlog::info("Setting LinkConfig to CXP6_X4...");
    setStringAndCheck<RemoteModule>("LinkConfig", "CXP6_X4");
    spdlog::info("LinkConfig set");
}

BehaviorCamera::~BehaviorCamera() {
    spdlog::debug("Behavior camera destructor called");
    cameraReadyFlag_.store(false);
}

void BehaviorCamera::start(size_t bufferSize) {
    frameGrabberPtr_->reallocBuffers(bufferSize);
    frameGrabberPtr_->start();
}

void BehaviorCamera::stop() {
    frameGrabberPtr_->stop();
}

FrameData BehaviorCamera::waitForOneFrame() {
    // Getting the buffer is the main blocking call
    Euresys::ScopedBuffer buffer(*frameGrabberPtr_);

    // Get image data and metadata
    uint64_t receivedTime = getCurrentTimeMicroseconds();
    uint8_t *dataPtr = buffer.getInfo<uint8_t *>(Euresys::gc::BUFFER_INFO_BASE);
    uint64_t grabberTimestamp =
        buffer.getInfo<uint64_t>(Euresys::gc::BUFFER_INFO_TIMESTAMP_NS);
    uint64_t acquisitionTime = grabberTimestamp / 1000;

    // Make FrameData object
    FrameData frameData;
    frameData.acquisitionTime = acquisitionTime;
    frameData.receivedTime = receivedTime;
    // Clone: dataPtr points into the grabber buffer owned by `buffer` (a
    // ScopedBuffer), which is requeued to the grabber when this function
    // returns. Without a copy the returned image would alias a buffer the
    // grabber may refill at any time.
    frameData.image =
        cv::Mat(imageHeight_, imageWidth_, CV_8UC1, dataPtr).clone();
    return frameData;
}

bool BehaviorCamera::isReady() const {
    return cameraReadyFlag_.load();
}

template <typename Module>
bool BehaviorCamera::setIntegerAndCheck(const std::string key, int value) {
    spdlog::info("Setting GenICam integer parameter {} to {}", key, value);
    frameGrabberPtr_->setInteger<Module>(key, value);
    int retrievedValue = frameGrabberPtr_->getInteger<Module>(key);
    if (retrievedValue != value) {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

template <typename Module>
bool BehaviorCamera::setStringAndCheck(
    const std::string key, const std::string value) {
    spdlog::info("Setting GenICam string parameter {} to {}", key, value);
    frameGrabberPtr_->setString<Module>(key, value);
    std::string retrievedValue = frameGrabberPtr_->getString<Module>(key);
    if (retrievedValue != value) {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

int roundToNearestValidBehaviorCamDimension(int value) {
    int remainder = value % 64;
    return value - remainder + (remainder < 32 ? 0 : 64);
}

std::tuple<int, int> getCenteredOffsets(
    int imageWidth, int imageHeight, int fullFrameWidth, int fullFrameHeight) {
    int xOffset = roundToNearestValidBehaviorCamDimension(
        (fullFrameWidth - imageWidth) / 2);
    int yOffset = roundToNearestValidBehaviorCamDimension(
        (fullFrameHeight - imageHeight) / 2);
    return std::make_tuple(xOffset, yOffset);
}
