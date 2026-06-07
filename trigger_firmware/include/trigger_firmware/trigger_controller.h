#pragma once

#include <deque>

#include <comm_protocol/protocol.h>

#include "trigger_firmware/device_io.h"
#include "trigger_firmware/serial_io.h"
#include "trigger_firmware/status_display.h"
#include "trigger_firmware/status_led.h"

/**
 * Top-level trigger controller: the firmware's main object. It wires together
 * serial command input (SerialIO), the physical outputs (DeviceIO), the OLED
 * status panel (StatusDisplay), and the RGB status LED (StatusLed), and runs
 * the acquisition timing loop.
 *
 * Lifecycle: construct once, call begin() from setup(), then call update() once
 * per loop() iteration. update() is non-blocking except for the deliberate
 * camera-buffer flush on START_RECORDING, so the timing loop runs at full
 * speed.
 *
 * Acquisition model (see docs/data_acquisition.md): the operating mode is
 * selected per parameter set by TriggerParams::enableMuscle.
 *
 * When enableMuscle is true (muscle-synced mode), the muscle camera free-runs
 * in continuous mode and exposes a status signal that is LOW during the common
 * time of every muscle frame. The controller waits for the onset of each common
 * time (DeviceIO::isMuscCommonTime() becoming true) and, on that edge, fires
 * the first behavior frame of a sync group together with the blue excitation
 * LED. It then fires the remaining behMuscSyncRatio - 1 behavior frames on its
 * own clock at behFrameRate before waiting for the next common-time onset. The
 * muscle camera is never triggered over TTL (it is open-loop), so the muscle
 * trigger pin stays idle.
 *
 * When enableMuscle is false (free-running mode), the muscle camera is ignored
 * entirely: the controller triggers the behavior camera on its own clock at
 * behFrameRate, never reads the muscle common-time signal, and never pulses the
 * blue excitation LED. The muscle-only parameters (muscEffExpTime,
 * behMuscSyncRatio, pcoCamRollingTime, pcoCamReadoutTime) are unused. This is
 * the mode for behavior-only acquisition.
 *
 * Commands (see docs/comm_protocol.md): STREAM and START_RECORDING reconfigure
 * the timing parameters; START_RECORDING additionally carries an opSequence
 * that toggles optogenetics channels at given behavior-frame indices and
 * reverts to streaming on its STOP step. STOP_RECORDING reverts an open
 * recording. LOG is echoed back over the serial port.
 *
 * Fault handling: a malformed command, a STOP_RECORDING received during a
 * scheduled recording, parameters whose camera exposure is not strictly shorter
 * than the frame period, or a detected muscle-frame overrun put the controller
 * into a latched error state. While in error every output is dropped and
 * triggering is halted; the controller leaves the error state only when the next
 * valid STREAM or START_RECORDING reconfigures it. A STOP_RECORDING with no open
 * recording to end (e.g. before any START_RECORDING) is instead logged as a
 * warning and ignored.
 */
class TriggerController {
  public:
    TriggerController();

    /** Initialize serial, outputs, the pause switch, and the OLED panel. */
    void begin();

    /** Advance the controller by one cycle; call every loop() iteration. */
    void update();

  private:
    // Logical operating mode. This is distinct from the displayed status, which
    // also reflects the pause override and the pre-configuration "initializing"
    // state.
    enum class Mode {
        streaming,
        openRecording,
        scheduledRecording,
    };

    // --- Command handling -------------------------------------------------
    void handleCommand(const Command &cmd);
    void handleStream(const Command &cmd);
    // Enter streaming mode with `params` (assumed timing-valid): clears the error
    // state and any opSequence, drops the opto channels, and refreshes status.
    // Shared by handleStream() and the default-streaming setup in begin().
    void startStreaming(const TriggerParams &params);
    void handleStartRecording(const Command &cmd);
    void handleStopRecording();
    void handleLog(const Command &cmd);

    // --- Timing engine ----------------------------------------------------
    // Dispatch one timing step to the active mode's path (selected by the most
    // recently applied params' enableMuscle flag).
    void runTriggers(unsigned long nowUs);
    // Muscle-synced mode (enableMuscle == true): lock each behavior sync group
    // to the muscle camera's common-time onset and pulse the blue LED.
    void runMuscleSyncedTriggers(unsigned long nowUs);
    // Free-running mode (enableMuscle == false): trigger the behavior camera on
    // the controller's own clock at behFrameRate, with no muscle sync, no blue
    // LED, and no overrun detection.
    void runFreeRunningTriggers(unsigned long nowUs);
    void processOpSequence();
    void revertToStreaming();
    void resetTiming();
    void applyParams(const TriggerParams &params);
    // True when both cameras' exposure times are strictly shorter than their
    // respective frame periods (the muscle frame period is syncRatio behavior
    // periods). Parameters that fail this check are rejected into the error
    // state.
    bool checkParamsTiming(const TriggerParams &params) const;

    // --- Pause switch and status output ----------------------------------
    void pollPauseSwitch();
    // Latch the error state: drop every output, log `reason` over serial, and
    // refresh the status outputs. Cleared by the next valid
    // STREAM/START_RECORDING.
    void enterError(const char *reason);
    bool isActive() const;
    StatusDisplay::Status currentStatus() const;
    // Push currentStatus() (and, when configured, the parameter lines) to both
    // the OLED panel and the RGB status LED.
    void refreshStatus();

    DeviceIO &device_;
    SerialIO serial_;
    StatusDisplay display_;
    StatusLed statusLed_;

    Mode mode_ = Mode::streaming;
    // True once parameters have been applied. begin() applies the default
    // streaming parameters, so the controller is configured (and streaming) from
    // startup; the "initializing" status is therefore only ever transient.
    bool configured_ = false;
    bool paused_ = false;     // physical on/off switch override
    bool error_ = false; // latched fault; halts triggering until reconfigured

    // Effective parameters and the timing values derived from them.
    TriggerParams params_;
    TriggerParams revertToParams_;
    bool muscleEnabled_ = true;           // params_.enableMuscle (active mode)
    unsigned long behPeriodUs_ = 1000000; // behavior frame period (us)
    unsigned int syncRatio_ = 1;          // behavior frames per muscle frame
    unsigned int behExpUs_ = 0;           // behavior exposure (us)
    unsigned int muscExpUs_ = 0;          // blue LED on-time (us)

    // Scheduled-recording operation sequence (empty => open recording).
    std::deque<OperationStep> opSequence_;

    // Timing-loop state.
    bool awaitingMuscEdge_ = true;   // waiting for the next common-time onset
    bool prevCommonTime_ = false;    // previous isMuscCommonTime() reading
    unsigned long groupStartUs_ = 0; // micros() at the current group's onset
    unsigned int frameInGroup_ = 0;  // next behavior frame index in the group
    unsigned long nextBehFrameUs_ = 0; // free-running mode: next frame due time
    bool behFrameActive_ = false;    // behavior shutter currently open
    unsigned long behFrameStartUs_ = 0;
    bool muscLEDActive_ = false; // blue excitation LED currently on
    unsigned long muscLEDStartUs_ = 0;
    unsigned long behFrameCount_ = 0; // behavior frames since recording start
};
