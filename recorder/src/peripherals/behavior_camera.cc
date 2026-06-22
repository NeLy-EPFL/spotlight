#include "recorder/peripherals/behavior_camera.h"

BehaviorCamera::BehaviorCamera(
    unsigned int image_width,
    unsigned int image_height,
    unsigned int x_offset,
    unsigned int y_offset,
    const std::string &io_line)
    : image_width_(image_width), image_height_(image_height),
      x_offset_(x_offset), y_offset_(y_offset), io_line_(io_line) {
    spdlog::info("Running GenTL eGrabber discovery...");
    Euresys::EGrabberDiscovery egrabber_discovery(gen_tl_);
    egrabber_discovery.discover();
    spdlog::info("GenTL eGrabber discovery completed");

    spdlog::info("Configuring camera...");
    camera_ = egrabber_discovery.cameras(0);
    frame_grabber_ptr_ = std::make_unique<Euresys::EGrabber<>>(camera_);
    Euresys::EGrabberInfo frame_grabber_info = camera_.grabbers[0];
    std::string interface_id = frame_grabber_info.interfaceID;
    std::string device_id = frame_grabber_info.deviceID;
    std::string device_vendor_name = frame_grabber_info.deviceVendorName;
    std::string device_model_name = frame_grabber_info.deviceModelName;
    spdlog::info(
        "Camera configured - interface ID: {}, device ID: {}, "
        "device vendor: {}, device model: {}",
        interface_id,
        device_id,
        device_vendor_name,
        device_model_name);

    configure();

    camera_ready_flag_.store(true);
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
    frame_grabber_ptr_->execute<InterfaceModule>("CxpPoCxpAuto");

    // Camera trigger + exposure. ExposureMode has to leave TriggerWidth before
    // the FrameStart trigger can be (re)configured and then be restored to
    // TriggerWidth; the guard is what makes a repeat call idempotent.
    set_string_and_check<RemoteModule>("TriggerSelector", "AcquisitionStart");
    set_string_and_check<RemoteModule>("TriggerMode", "On");
    set_string_and_check<RemoteModule>("TriggerSource", "CXPin");
    set_string_and_check<RemoteModule>("TriggerSelector", "FrameStart");
    if (frame_grabber_ptr_->getString<RemoteModule>("ExposureMode") ==
        "TriggerWidth") {
        set_string_and_check<RemoteModule>("ExposureMode", "Off");
    }
    set_string_and_check<RemoteModule>("TriggerMode", "On");
    set_string_and_check<RemoteModule>("TriggerSource", "CXPin");
    set_string_and_check<RemoteModule>("ExposureMode", "TriggerWidth");
    set_string_and_check<RemoteModule>("LinkConfig", "CXP6_X4");

    // Camera control method. "RG" is overridden to "EXTERNAL" below;
    // CycleTriggerSource is left as set here.
    set_string_and_check<DeviceModule>("CameraControlMethod", "RG");
    set_string_and_check<DeviceModule>("CycleTriggerSource", "Immediate");

    // Strobe output lines. TTLIO12 is overridden to a trigger input below (it
    // carries the external frame trigger from the microcontroller).
    set_string_and_check<InterfaceModule>("LineSelector", "TTLIO11");
    set_string_and_check<InterfaceModule>("LineSource", "Device0Strobe");
    set_string_and_check<InterfaceModule>("LineMode", "Output");
    set_string_and_check<InterfaceModule>("LineSelector", "TTLIO12");
    set_string_and_check<InterfaceModule>("LineSource", "Device0Strobe");
    set_string_and_check<InterfaceModule>("LineMode", "Output");
    // Numeric features: set without the strict string read-back check, since
    // they read back in a different textual form (e.g. "2" -> "2.000000").
    spdlog::info("Setting LineSourceDivisionFactor to 4 and Gain to 2");
    frame_grabber_ptr_->setString<InterfaceModule>(
        "LineSourceDivisionFactor", "4");
    frame_grabber_ptr_->setString<RemoteModule>("Gain", "2");

    // =======================================================================
    // Sensor ROI from the recorder config (supersedes the script's hard-coded
    // ROI). Set the offsets to 0 first - if the new image size is larger than
    // the current one, setting the new size directly may fail when
    // newSize + currentOffset exceeds the sensor bounds.
    // =======================================================================
    spdlog::info("Setting sensor OffsetX and OffsetY to 0, 0");
    set_integer_and_check<RemoteModule>("OffsetX", 0);
    set_integer_and_check<RemoteModule>("OffsetY", 0);

    spdlog::info(
        "Setting sensor ROI - width: {}, height: {}, "
        "x_offset: {}, y_offset: {}",
        image_width_,
        image_height_,
        x_offset_,
        y_offset_);
    set_integer_and_check<RemoteModule>("Width", image_width_);
    set_integer_and_check<RemoteModule>("Height", image_height_);
    set_integer_and_check<RemoteModule>("OffsetX", x_offset_);
    set_integer_and_check<RemoteModule>("OffsetY", y_offset_);
    spdlog::info("Sensor ROI set");

    // =======================================================================
    // External-trigger configuration. The microcontroller feeds a trigger
    // signal (high = active, exposure time controlled by trigger width) into
    // `ioLine_` (TTLIO12) of the frame grabber.
    // =======================================================================
    spdlog::info("Configuring trigger-related settings...");
    spdlog::info("Setting {} line as Input...", io_line_);
    set_string_and_check<InterfaceModule>("LineSelector", io_line_);
    set_string_and_check<InterfaceModule>("LineMode", "Input");

    spdlog::info("Setting LIN1 to specified ioLine ({})...", io_line_);
    set_string_and_check<InterfaceModule>("LineInputToolSelector", "LIN1");
    set_string_and_check<InterfaceModule>("LineInputToolSource", io_line_);

    spdlog::info("Setting CameraControlMethod to EXTERNAL...");
    set_string_and_check<DeviceModule>("CameraControlMethod", "EXTERNAL");

    spdlog::info("Disabling trigger for AcquisitionStart/AcquisitionEnd...");
    set_string_and_check<RemoteModule>("TriggerSelector", "AcquisitionStart");
    set_string_and_check<RemoteModule>("TriggerMode", "Off");
    set_string_and_check<RemoteModule>("TriggerSelector", "AcquisitionEnd");
    set_string_and_check<RemoteModule>("TriggerMode", "Off");

    spdlog::info("Enabling trigger for FrameStart, using CXPin as source...");
    set_string_and_check<RemoteModule>("TriggerSelector", "FrameStart");
    // TriggerMode must be "On" for FrameStart when CameraControlMethod is
    // EXTERNAL (the option is grayed out / forced on), so it is checked rather
    // than set here.
    assert(frame_grabber_ptr_->getString<RemoteModule>("TriggerMode") == "On");
    set_string_and_check<RemoteModule>("TriggerSource", "CXPin");
    spdlog::info("Trigger-related settings configured");

    spdlog::info("Setting LinkConfig to CXP6_X4...");
    set_string_and_check<RemoteModule>("LinkConfig", "CXP6_X4");
    spdlog::info("LinkConfig set");
}

BehaviorCamera::~BehaviorCamera() {
    spdlog::debug("Behavior camera destructor called");
    camera_ready_flag_.store(false);
}

void BehaviorCamera::start(size_t buffer_size) {
    frame_grabber_ptr_->reallocBuffers(buffer_size);
    frame_grabber_ptr_->start();
}

void BehaviorCamera::stop() {
    frame_grabber_ptr_->stop();
}

FrameData BehaviorCamera::wait_for_one_frame() {
    // Getting the buffer is the main blocking call
    Euresys::ScopedBuffer buffer(*frame_grabber_ptr_);

    // Get image data and metadata
    uint64_t received_time = get_current_time_microseconds();
    uint8_t *data_ptr =
        buffer.getInfo<uint8_t *>(Euresys::gc::BUFFER_INFO_BASE);
    uint64_t grabber_timestamp =
        buffer.getInfo<uint64_t>(Euresys::gc::BUFFER_INFO_TIMESTAMP_NS);
    uint64_t acquisition_time = grabber_timestamp / 1000;

    // Make FrameData object
    FrameData frame_data;
    frame_data.acquisition_time = acquisition_time;
    frame_data.received_time = received_time;
    // Clone: data_ptr points into the grabber buffer owned by `buffer` (a
    // ScopedBuffer), which is requeued to the grabber when this function
    // returns. Without a copy the returned image would alias a buffer the
    // grabber may refill at any time.
    frame_data.image =
        cv::Mat(image_height_, image_width_, CV_8UC1, data_ptr).clone();
    return frame_data;
}

bool BehaviorCamera::is_ready() const {
    return camera_ready_flag_.load();
}

template <typename Module>
bool BehaviorCamera::set_integer_and_check(const std::string &key, int value) {
    spdlog::info("Setting GenICam integer parameter {} to {}", key, value);
    frame_grabber_ptr_->setInteger<Module>(key, value);
    int retrieved_value = frame_grabber_ptr_->getInteger<Module>(key);
    if (retrieved_value != value) {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

template <typename Module>
bool BehaviorCamera::set_string_and_check(
    const std::string &key, const std::string &value) {
    spdlog::info("Setting GenICam string parameter {} to {}", key, value);
    frame_grabber_ptr_->setString<Module>(key, value);
    std::string retrieved_value = frame_grabber_ptr_->getString<Module>(key);
    if (retrieved_value != value) {
        spdlog::error("Failed to set GenICam parameter {} to {}", key, value);
        return false;
    }
    return true;
}

int round_to_nearest_valid_behavior_cam_dimension(int value) {
    int remainder = value % 64;
    return value - remainder + (remainder < 32 ? 0 : 64);
}

std::tuple<int, int> get_centered_offsets(
    int image_width,
    int image_height,
    int full_frame_width,
    int full_frame_height) {
    int x_offset = round_to_nearest_valid_behavior_cam_dimension(
        (full_frame_width - image_width) / 2);
    int y_offset = round_to_nearest_valid_behavior_cam_dimension(
        (full_frame_height - image_height) / 2);
    return std::make_tuple(x_offset, y_offset);
}
