#include "recorder/peripherals/muscle_camera.h"

// The PCO headers select their Linux code paths with `#elif PCO_LINUX`, so
// PCO_LINUX must expand to a non-empty token (an empty define would produce
// `#elif` with no expression). The build provides it per-file via
// set_source_files_properties on this .cc (and the bundled PCO SDK sources);
// this is a fallback so the file still compiles if that is ever dropped.
#ifndef PCO_LINUX
#define PCO_LINUX 1
#endif

#include <atomic>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <mutex>

#include <opencv2/opencv.hpp>

// clang-format off
// stdafx.h must come first: it includes pco_linux_defs.h (WORD/BYTE/DWORD) and
// <variant>, which camera.h, cameraexception.h, and sc2_defs.h all depend on.
// Disable clang-format, which would sort these includes alphabetically.
#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"
#include "sc2_defs.h"
// clang-format on

namespace {
// Wait for an image with a small internal delay and a short timeout, so the
// blocking grab returns regularly to re-check the stop flag (see
// Impl::waitForOneFrame). Mirrors the values used by the old PCO server.
constexpr bool kWaitWithSmallDelay = true;
constexpr double kWaitTimeoutSecs = 0.1;
constexpr uint32_t kTimeoutErrorCode = 0x80004001; // see PCO manual
// Ring-buffer depth for continuous recording.
constexpr int kRecordBufferSize = 10;
} // namespace

// ===========================================================================
// MuscleCamera::Impl -- all PCO SDK state and the acquire loop. This struct is
// the whole reason MuscleCamera is pimpl'd: it is defined only in this .cc (the
// lone TU compiled with PCO_LINUX=1), so the PCO headers and their global
// Windows-ism shims (BOOL/WORD/DWORD/...) never reach muscle_camera.h or anything
// that includes it. See the class comment in muscle_camera.h for the rationale.
// ===========================================================================
struct MuscleCamera::Impl {
    // ROI (1-based sensor coordinates, as the PCO SDK expects).
    unsigned int x0;
    unsigned int x1;
    unsigned int y0;
    unsigned int y1;
    int imageWidth;
    int imageHeight;
    double sensorReadoutTimeUs;

    // Set by stop() (any thread), observed by the acquirer thread.
    std::atomic<bool> stopRequested{false};

    // Desired state posted by setNominalExposureUs()/setEnabled() (any thread)
    // and applied by the acquirer thread before its next grab. Guarded by
    // paramMutex; paramCv wakes the acquirer when it is blocked waiting (camera
    // disabled) and the state changes (re-enabled or stop requested).
    std::mutex paramMutex;
    std::condition_variable paramCv;
    unsigned int desiredExposureUs = 0;
    bool desiredEnabled = true;

    // Acquirer-thread-only state.
    pco::Camera camera;
    unsigned int currentExposureUs = 0;
    bool recording = false;
    bool isFirstFrame = true;

    Impl(
        int imageWidth_,
        int imageHeight_,
        int xOffset,
        int yOffset,
        double sensorReadoutTimeUs_,
        unsigned int initialExposureUs)
        : x0(xOffset + 1), x1(xOffset + imageWidth_), y0(yOffset + 1),
          y1(yOffset + imageHeight_), imageWidth(imageWidth_),
          imageHeight(imageHeight_), sensorReadoutTimeUs(sensorReadoutTimeUs_),
          desiredExposureUs(initialExposureUs),
          currentExposureUs(initialExposureUs) {}

    // Apply the static (ROI / trigger / SMA#4) configuration. Ported one-to-one
    // from the old pco-camera-server's setupPCOCamera.
    void configureCamera() {
        spdlog::info("Getting default PCO camera configuration");
        camera.defaultConfiguration();
        pco::Configuration config = camera.getConfiguration();
        config.roi.x0 = x0;
        config.roi.x1 = x1;
        config.roi.y0 = y0;
        config.roi.y1 = y1;
        // Auto-sequence ("auto trigger") = continuous rolling shutter: the camera
        // free-runs, exposing each line back-to-back with no idle line-reset
        // time, instead of waiting for an external TTL trigger per frame. This is
        // required by the acquisition design (docs/data_acquisition.md): the
        // trigger firmware does NOT trigger this camera -- it locks the behavior
        // camera to the muscle camera's free-running common-time signal on
        // SMA #4 (configured below). The free-run frame rate is set via the
        // nominal exposure (see applyExposure).
        config.trigger_mode = TRIGGER_MODE_AUTOTRIGGER;
        config.acquire_mode = ACQUIRE_MODE_AUTO;
        // Zero inter-frame delay keeps the rolling shutter continuous (no idle
        // time). Any sync delay is implemented in the trigger firmware, not here.
        config.delay_time_s = 0.0;
        config.noise_filter_mode = NOISE_FILTER_MODE_ON;
        spdlog::info(
            "Setting PCO camera configuration: x0={}, x1={}, y0={}, y1={}",
            config.roi.x0,
            config.roi.x1,
            config.roi.y0,
            config.roi.y1);
        camera.setConfiguration(config);

        // Set trigger polarity.
        camera.configureHWIO_1_exposureTrigger(
            true, pco::HWIO_EdgePolarity::rising_edge);

        // Drive SMA #4 as the muscle camera's "common time" reference for the
        // trigger firmware. The firmware treats the line being HIGH as "in common
        // time" and fires the behavior frame + blue LED on the LOW->HIGH onset,
        // so the camera must drive the line HIGH for exactly the common-time
        // window.
        //   - signal_type status_expos: report the exposure status on SMA #4.
        //   - timing global: for a rolling shutter, "global" is the interval when
        //     all lines are exposed simultaneously, i.e. the common time. NOT
        //     all_lines, which spans the whole rolling exposure envelope and would
        //     make the onset fire ~rollingTime too early.
        //   - polarity high_level: makes the line HIGH during common time (LOW
        //     otherwise), matching the firmware's edge.
        camera.configureHWIO_4_statusExpos(
            true,
            pco::HWIO_Polarity::high_level,
            pco::HWIO_4_SignalType::status_expos,
            pco::HWIO_StatusExpos_Timing::global); // global = common time
    }

    // Program the nominal per-line exposure (also the free-run frame rate). Must
    // be called while NOT recording (PCO does not allow changing the free-run
    // exposure during recording -- hence the stop->reconfigure->restart in
    // applyPendingReconfig).
    void applyExposure(unsigned int exposureUs) {
        spdlog::info("Setting PCO camera nominal exposure to {} us", exposureUs);
        camera.setExposureTime(exposureUs / 1000000.0);
        camera.autoExposureOff();
        currentExposureUs = exposureUs;
    }

    void startRecording() {
        camera.record(kRecordBufferSize, pco::RecordMode::ring_buffer);
        recording = true;
        // After a (re)start the next image is again a "first image".
        isFirstFrame = true;
    }

    void stopRecording() {
        if (recording) {
            camera.stop();
            recording = false;
        }
    }

    // Bring the camera in line with the desired enable/exposure state, via
    // stop->reconfigure->restart as needed (the PCO camera cannot change its
    // free-run exposure, nor start/stop, without stopping the recording). Runs on
    // the acquirer thread only. Returns true if the camera is now enabled and
    // recording (ready to grab), false if it is currently disabled.
    bool applyDesiredState() {
        bool enabled;
        unsigned int exposureUs;
        {
            std::lock_guard<std::mutex> lock(paramMutex);
            enabled = desiredEnabled;
            exposureUs = desiredExposureUs;
        }
        if (!enabled) {
            stopRecording();
            return false;
        }
        // Enabled: a live exposure change requires stop->reprogram->restart, so
        // stop first if the desired exposure differs from what is programmed.
        if (recording && exposureUs != currentExposureUs) {
            spdlog::info(
                "Applying muscle camera exposure change {} us -> {} us "
                "(stop/reconfigure/restart)",
                currentExposureUs,
                exposureUs);
            stopRecording();
        }
        if (!recording) {
            applyExposure(exposureUs);
            startRecording();
        }
        return true;
    }

    std::optional<FrameData> waitForOneFrame() {
        pco::Image pcoImage;
        while (!stopRequested.load()) {
            // Apply any pending exposure/enable change, (re)starting recording as
            // needed. If the camera is disabled, block until it is re-enabled or
            // stop is requested (rather than busy-spinning), then report "no
            // frame" so the acquirer loop can re-check its own shutdown flag.
            if (!applyDesiredState()) {
                std::unique_lock<std::mutex> lock(paramMutex);
                paramCv.wait_for(
                    lock, std::chrono::milliseconds(200), [this] {
                        return stopRequested.load() || desiredEnabled;
                    });
                return std::nullopt;
            }

            // Blocking grab with a short timeout so we periodically return to
            // the top of the loop and can observe stopRequested / a pending
            // reconfigure. A timeout is expected (e.g. when the muscle camera is
            // running slowly) and is not an error.
            try {
                if (isFirstFrame) {
                    camera.waitForFirstImage(
                        kWaitWithSmallDelay, kWaitTimeoutSecs);
                    isFirstFrame = false;
                } else {
                    camera.waitForNewImage(
                        kWaitWithSmallDelay, kWaitTimeoutSecs);
                }
            } catch (pco::CameraException &e) {
                if (static_cast<uint32_t>(e.error_code()) == kTimeoutErrorCode) {
                    continue; // expected; re-check stop flag and try again
                }
                throw;
            }

            // Fetch the latest image and copy it out. The cv::Mat wraps the SDK's
            // buffer, which is reused on the next grab, so clone() takes a private
            // copy before returning.
            camera.image(
                pcoImage, PCO_RECORDER_LATEST_IMAGE, pco::DataFormat::Mono16);
            cv::Mat cvImage(
                pcoImage.height(),
                pcoImage.width(),
                CV_16UC1,
                pcoImage.raw_data().first);

            FrameData frameData;
            frameData.acquisitionTime = getCurrentTimeMicroseconds();
            frameData.receivedTime = frameData.acquisitionTime;
            frameData.image = cvImage.clone();
            return frameData;
        }
        return std::nullopt;
    }
};

namespace {
bool isROIValid(
    int x0,
    int x1,
    int y0,
    int y1,
    int imageWidth,
    int imageHeight,
    const RecorderConfig &recorderConfig) {
    int fullFrameWidth =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_width");
    int fullFrameHeight =
        recorderConfig.getParameter<int>("muscle_camera", "full_frame_height");
    if (x0 < 1 || x1 > fullFrameWidth || y0 < 1 || y1 > fullFrameHeight ||
        x0 >= x1 || y0 >= y1 || imageWidth % 32 != 0 || imageHeight % 8 != 0 ||
        imageWidth < 64 || imageHeight < 16) {
        spdlog::critical(
            "Invalid ROI for muscle camera. The following conditions must be "
            "met: 1 <= x0 < x1 <= {}; 1 <= y0 < y1 <= {}. Furthermore, the "
            "minimum size of the ROI is 64x16 pixels. The width must be a "
            "multiple of 32 and the height must be a multiple of 8.",
            fullFrameWidth,
            fullFrameHeight);
        return false;
    }
    return true;
}

// Initial nominal per-line exposure for the free-running camera, derived from
// the default streaming muscle interval (streaming sync ratio / streaming
// behavior FPS): exposure = muscleInterval - readout. The GUI overwrites this
// live (via setNominalExposureUs) as soon as it knows the active parameters.
unsigned int computeDefaultExposureUs(
    const RecorderConfig &recorderConfig, double sensorReadoutTimeUs) {
    const int streamingBehFPS = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    const int streamingSyncRatio = recorderConfig.getParameter<int>(
        "muscle_camera", "streaming_sync_ratio");
    const unsigned int defaultMuscleIntervalUs = static_cast<unsigned int>(
        1000000.0 * streamingSyncRatio / streamingBehFPS);
    return defaultMuscleIntervalUs -
        static_cast<unsigned int>(sensorReadoutTimeUs);
}
} // namespace

MuscleCamera::MuscleCamera(
    int imageWidth,
    int imageHeight,
    int xOffset,
    int yOffset,
    double sensorReadoutTimeUs,
    const RecorderConfig &recorderConfig) {
    int x0 = xOffset + 1;
    int x1 = xOffset + imageWidth;
    int y0 = yOffset + 1;
    int y1 = yOffset + imageHeight;
    if (!isROIValid(
            x0, x1, y0, y1, imageWidth, imageHeight, recorderConfig)) {
        throw std::runtime_error("Invalid ROI for muscle camera");
    }

    // The PCO SDK keeps global state (camera scan/open handles, recorder, etc.)
    // that must be initialized before any pco::Camera is constructed. Skipping
    // this makes the Camera constructor's PCO_ScanCameras/PCO_OpenCameraDevice
    // calls operate on an invalid SDK handle, which surfaces as the cryptic
    // "Handle is invalid" (0xa00a3002). Mirror the PCO samples, which always pair
    // PCO_InitializeLib()/PCO_CleanupLib() around camera use.
    spdlog::info("Initializing PCO SDK library");
    if (int err = PCO_InitializeLib(); err != PCO_NOERROR) {
        throw std::runtime_error(fmt::format(
            "Failed to initialize PCO SDK library (error 0x{:08x})",
            static_cast<uint32_t>(err)));
    }

    unsigned int initialExposureUs =
        computeDefaultExposureUs(recorderConfig, sensorReadoutTimeUs);

    try {
        impl_ = std::make_unique<Impl>(
            imageWidth,
            imageHeight,
            xOffset,
            yOffset,
            sensorReadoutTimeUs,
            initialExposureUs);
        spdlog::info("Setting up PCO muscle camera");
        impl_->configureCamera();
        impl_->applyExposure(initialExposureUs);
        impl_->startRecording();
        spdlog::info("PCO muscle camera setup complete; recording started");
    } catch (...) {
        // The Impl (and its pco::Camera) is destroyed by unique_ptr; release the
        // SDK so a later run can reopen the camera.
        impl_.reset();
        PCO_CleanupLib();
        throw;
    }
}

MuscleCamera::~MuscleCamera() {
    // The acquirer thread has already been joined by the time we get here (see
    // the threading contract in the header), so it is safe to touch the camera.
    if (impl_) {
        try {
            impl_->stopRecording();
        } catch (const std::exception &e) {
            spdlog::warn("Error stopping PCO muscle camera: {}", e.what());
        }
        impl_.reset();
    }
    spdlog::info("PCO muscle camera stopped; releasing SDK");
    PCO_CleanupLib();
}

void MuscleCamera::stop() {
    {
        std::lock_guard<std::mutex> lock(impl_->paramMutex);
        impl_->stopRequested.store(true);
    }
    // Wake the acquirer if it is blocked in the disabled-state wait.
    impl_->paramCv.notify_all();
}

void MuscleCamera::setEnabled(bool enabled) {
    {
        std::lock_guard<std::mutex> lock(impl_->paramMutex);
        impl_->desiredEnabled = enabled;
    }
    // Wake the acquirer if it is parked in the disabled-state wait so it can
    // restart the camera promptly on re-enable.
    impl_->paramCv.notify_all();
}

std::optional<FrameData> MuscleCamera::waitForOneFrame() {
    return impl_->waitForOneFrame();
}

void MuscleCamera::setNominalExposureUs(unsigned int exposureUs) {
    // Just post the desired value; the acquirer thread compares it against the
    // programmed exposure each loop and applies any change via
    // stop->reconfigure->restart (see Impl::applyDesiredState).
    std::lock_guard<std::mutex> lock(impl_->paramMutex);
    impl_->desiredExposureUs = exposureUs;
}

int MuscleCamera::getNumLinesScanned() const {
    return impl_->imageHeight;
}

// MuscleTriggerTiming and the ROI-rounding helpers are hardware-independent (no
// PCO calls), but live here alongside the rest of muscle_camera.h's
// implementation so there is one .cc per header. The recorder unit tests link
// this TU (and the PCO SDK) to exercise them; that is fine because the tests
// never open or talk to a camera (see recorder/tests/CMakeLists.txt).

int roundToNearestValidMuscleCamHorizontal(int value) {
    int remainder = value % 32;
    return value - remainder + (remainder < 16 ? 0 : 32);
}

int roundToNearestValidMuscleCamVertical(int value) {
    int remainder = value % 8;
    return value - remainder + (remainder < 4 ? 0 : 8);
}

bool MuscleTriggerTiming::computeParameters(
    int muscleImageHeight,
    double muscleCameraLineScanTimeUs,
    int muscleCameraReadoutTimeUs) {
    double muscleCameraFPS = behaviorCameraFPS_ / double(syncRatio_);
    int muscleIntervalUs = 1000000 / muscleCameraFPS;
    int rollingTimeUs = muscleImageHeight * muscleCameraLineScanTimeUs;

    // Continuous rolling shutter (auto-sequence): the camera free-runs at
    // 1/(nominalExposure + readout). Set the nominal per-line exposure so one
    // frame fills the requested muscle interval, then split it into the rolling
    // time (line skew) and the common time (all lines exposing simultaneously).
    // The light-on window must fit inside the common time, leaving a
    // non-negative buffer:
    //   nominalExposure = muscleInterval - readout = rollingTime + commonTime
    //   commonTime      = lightOn + bufferTime
    // Valid only if rollingTime + lightOn + readout <= muscleInterval. (Note the
    // single rollingTime: triggered acquisition would need 2 * rollingTime, but
    // continuous rolling has no idle line-reset time -- see
    // docs/data_acquisition.md.)
    nominalExposureUs_ = muscleIntervalUs - muscleCameraReadoutTimeUs;
    commonTimeUs_ = nominalExposureUs_ - rollingTimeUs;
    bufferTimeUs_ = commonTimeUs_ - muscleLightOnTimeUs_;
    if (bufferTimeUs_ < 0) {
        spdlog::critical(
            "Computed muscle camera parameters are invalid: "
            "rollingTime + lightOn + readout must be <= muscleInterval. "
            "rollingTimeUs = {}, "
            "muscleCameraReadoutTimeUs = {}, "
            "muscleLightOnTimeUs = {}, "
            "muscleIntervalUs = {}",
            rollingTimeUs,
            muscleCameraReadoutTimeUs,
            muscleLightOnTimeUs_,
            muscleIntervalUs);
        return false; // Invalid configuration
    }
    return true;
}
