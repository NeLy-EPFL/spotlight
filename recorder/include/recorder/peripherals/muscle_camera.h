#pragma once

#include <memory>
#include <optional>
#include <string>

#include <spdlog/spdlog.h>

#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

// Derives the muscle camera's continuous-mode (auto-sequence) exposure from the
// recording parameters and validates that the requested muscle frame interval is
// long enough to fit the rolling shutter, light-on, and sensor readout times.
//
// In continuous mode the camera free-runs at 1/(nominalExposure + readout), so
// the nominal per-line exposure programmed into the camera is set so that one
// frame fills the desired muscle frame interval:
//   nominalExposure = muscleInterval - readout = rollingTime + commonTime
//   commonTime      = lightOn + bufferTime   (the SMA#4 common-time HIGH window)
// The blue excitation LED is pulsed for lightOn within the common time, which the
// trigger firmware locks to via the SMA#4 status line. See
// docs/data_acquisition.md.
class MuscleTriggerTiming {
  public:
    MuscleTriggerTiming(
        int behaviorCameraFPS, int syncRatio, int muscleLightOnTimeUs)
        : behaviorCameraFPS_(behaviorCameraFPS), syncRatio_(syncRatio),
          muscleLightOnTimeUs_(muscleLightOnTimeUs) {}

    // Populates the derived getters below. Returns false if the configuration
    // is invalid (the muscle frame interval is too short to fit the light-on
    // window inside the common time, i.e. the buffer time would be negative).
    bool computeParameters(
        int muscleImageHeight,
        double muscleCameraLineScanTimeUs,
        int muscleCameraReadoutTimeUs);

    // Nominal per-line exposure to program into the camera (us).
    int getNominalExposureUs() const {
        return nominalExposureUs_;
    }
    // Slack in the common-time window beyond the light-on time (us).
    int getBufferTimeUs() const {
        return bufferTimeUs_;
    }
    // Duration of the common time / SMA#4 HIGH window (us).
    int getCommonTimeUs() const {
        return commonTimeUs_;
    }

  private:
    // User input parameters
    int behaviorCameraFPS_;
    int syncRatio_;
    int muscleLightOnTimeUs_;

    // Derived parameters
    int nominalExposureUs_ = -1;
    int bufferTimeUs_ = -1;
    int commonTimeUs_ = -1;
};

// In-process driver for the PCO muscle camera. Mirrors BehaviorCamera: the
// camera is opened in the constructor and driven directly (no separate process,
// no shared memory).
//
// ===========================================================================
// Why this class is pimpl'd (the PCO SDK is hidden behind an opaque Impl)
// ===========================================================================
// The PCO SDK headers (camera.h, sc2_defs.h, ...) only compile on Linux when
// PCO_LINUX is defined, and that switch makes pco_linux_defs.h inject a pile of
// Windows-compatibility typedefs and macros -- BOOL, BYTE, WORD, DWORD, HANDLE,
// FALSE/TRUE, MAKEWORD, far, MAX_PATH, ... -- into the GLOBAL namespace of every
// translation unit that includes a PCO header. Those shims have no include
// guards and are not namespaced, so they leak everywhere a PCO header is pulled
// in.
//
// We do not want that pollution (nor the PCO include directories, nor the
// per-file PCO_LINUX=1 requirement) anywhere except the single .cc that actually
// talks to the camera. This header is included -- transitively, via
// muscle_recording.h -- by GUI, tracking, and unit-test translation units that
// have nothing to do with the PCO SDK; dragging the PCO headers (and their
// Windows-isms) into all of them would be fragile and pointless.
//
// The pimpl idiom solves this: every PCO type lives inside `struct Impl`, which
// is *defined* only in muscle_camera.cc (the lone TU that includes PCO headers,
// compiled with per-file PCO_LINUX=1; see recorder/CMakeLists.txt). This header
// forward-declares Impl and holds it via std::unique_ptr, so it stays completely
// PCO-free and can be included freely by any code. The cost is one pointer
// indirection per call -- negligible next to a camera grab.
//
// ===========================================================================
// Threading contract
// ===========================================================================
//   - waitForOneFrame() must be called from a single thread only (the muscle
//     image acquirer thread). It is the *only* place the PCO SDK is touched,
//     including the stop->reconfigure->restart used to apply a pending exposure
//     change or an enable/disable. This keeps the vendor SDK single-threaded
//     with no locking on the hot path.
//   - stop(), setNominalExposureUs() and setEnabled() are thread-safe and may be
//     called from other threads (the GUI / shutdown path). They only set
//     flags/values that the acquirer thread observes; they never touch the
//     camera themselves.
//   - The MuscleCamera object MUST outlive its acquirer thread. stop() makes a
//     blocked waitForOneFrame() return std::nullopt promptly so the acquirer can
//     exit and be joined before the object is destroyed (~MuscleCamera closes
//     the camera and tears down the SDK).
class MuscleCamera {
  public:
    MuscleCamera(
        int imageWidth,
        int imageHeight,
        int xOffset,
        int yOffset,
        double sensorReadoutTimeUs,
        const RecorderConfig &recorderConfig);
    ~MuscleCamera();

    // Owns the PCO camera + SDK lifetime; not copyable or movable.
    MuscleCamera(const MuscleCamera &) = delete;
    MuscleCamera &operator=(const MuscleCamera &) = delete;

    // Block until the next frame is acquired and return it, applying any pending
    // exposure / enable-state change first. Returns std::nullopt when there is no
    // frame to return -- because stop() has been called (shutdown) or because the
    // camera is currently disabled (see setEnabled). Callers distinguish the two
    // via their own shutdown flag (the muscle acquirer checks programState's
    // toQuit). While disabled this blocks briefly rather than spinning. Must be
    // called from a single thread.
    std::optional<FrameData> waitForOneFrame();

    // Request that acquisition stop and any in-progress (or future)
    // waitForOneFrame() return std::nullopt. This is the terminal shutdown
    // signal (not the same as disabling via setEnabled). Thread-safe and
    // idempotent; does not block on the acquirer or touch the camera.
    void stop();

    // Enable or disable acquisition. When disabled, the acquirer thread stops the
    // camera (it no longer free-runs or drives its common-time signal) and
    // waitForOneFrame() returns std::nullopt; when re-enabled the camera is
    // restarted. Wired to the GUI's "Enable muscle imaging" checkbox. Thread-safe;
    // the change is applied by the acquirer thread before its next grab. Enabled
    // by default.
    void setEnabled(bool enabled);

    // Post a new nominal per-line exposure (us), to be applied by the acquirer
    // thread before its next grab via stop->reconfigure->restart (the PCO camera
    // cannot change its free-run exposure/frame rate while recording). In
    // continuous (auto-sequence) mode this also sets the free-run frame rate,
    // since the camera runs at 1/(nominalExposure + readout). Thread-safe. Derive
    // the value with MuscleTriggerTiming::getNominalExposureUs().
    void setNominalExposureUs(unsigned int exposureUs);

    int getNumLinesScanned() const;

  private:
    // PCO SDK state lives behind a pimpl so this header stays PCO-free (see the
    // class comment above for the full rationale).
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

int roundToNearestValidMuscleCamHorizontal(int value);
int roundToNearestValidMuscleCamVertical(int value);
