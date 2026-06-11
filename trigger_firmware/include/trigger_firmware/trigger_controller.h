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
 * selected per parameter set by TriggerParams::enable_muscle.
 *
 * When enable_muscle is true (muscle-synced mode), the muscle camera free-runs
 * in continuous mode and exposes a status signal that is HIGH during the common
 * time of every muscle frame. The controller waits for the onset of each common
 * time (DeviceIO::is_musc_common_time() becoming true) and, on that edge, fires
 * the first behavior frame of a sync group together with the blue excitation
 * LED. It then fires the remaining beh_musc_sync_ratio - 1 behavior frames on
 * its own clock at beh_frame_rate before waiting for the next common-time
 * onset. The muscle camera is never triggered over TTL (it is open-loop), so
 * the muscle trigger pin stays idle.
 *
 * When enable_muscle is false (free-running mode), the muscle camera is ignored
 * entirely: the controller triggers the behavior camera on its own clock at
 * beh_frame_rate, never reads the muscle common-time signal, and never pulses
 * the blue excitation LED. The muscle-only parameters (musc_eff_exp_time,
 * beh_musc_sync_ratio, pco_cam_rolling_time, pco_cam_readout_time) are unused.
 * This is the mode for behavior-only acquisition.
 *
 * Commands (see docs/comm_protocol.md): STREAM and START_RECORDING reconfigure
 * the timing parameters; START_RECORDING additionally carries an op_sequence
 * that toggles optogenetics channels at given behavior-frame indices and
 * reverts to streaming on its STOP step. STOP_RECORDING reverts an open
 * recording. LOG is echoed back over the serial port. RESET reboots the
 * microcontroller (esp_restart()); the recorder sends it at the start of every
 * program so the controller always begins from a clean, known state.
 *
 * Fault handling: a malformed command, a STOP_RECORDING received during a
 * scheduled recording, parameters whose camera exposure is not strictly shorter
 * than the frame period, or a detected muscle-frame overrun put the controller
 * into a latched error state. While in error every output is dropped and
 * triggering is halted; the controller leaves the error state only when the
 * next valid STREAM or START_RECORDING reconfigures it. A STOP_RECORDING with
 * no open recording to end (e.g. before any START_RECORDING) is instead logged
 * as a warning and ignored.
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
        open_recording,
        scheduled_recording,
    };

    // --- Command handling -------------------------------------------------
    void handle_command(const Command &cmd);
    void handle_stream(const Command &cmd);
    // Enter streaming mode with `params` (assumed timing-valid): clears the
    // error state and any op_sequence, drops the opto channels, and refreshes
    // status. Shared by handle_stream() and the default-streaming setup in
    // begin().
    void start_streaming(const TriggerParams &params);
    void handle_start_recording(const Command &cmd);
    void handle_stop_recording();
    void handle_log(const Command &cmd);
    // Show "RESETTING" on the status outputs, then reboot the MCU via
    // esp_restart(); this call does not return.
    void handle_reset();

    // --- Timing engine ----------------------------------------------------
    // Dispatch one timing step to the active mode's path (selected by the most
    // recently applied params' enable_muscle flag).
    void run_triggers(unsigned long now_us);
    // Muscle-synced mode (enable_muscle == true): lock each behavior sync group
    // to the muscle camera's common-time onset and pulse the blue LED.
    void run_muscle_synced_triggers(unsigned long now_us);
    // Free-running mode (enable_muscle == false): trigger the behavior camera
    // on the controller's own clock at beh_frame_rate, with no muscle sync, no
    // blue LED, and no overrun detection.
    void run_free_running_triggers(unsigned long now_us);
    void process_op_sequence();
    void revert_to_streaming();
    void reset_timing();
    void apply_params(const TriggerParams &params);
    // True when both cameras' exposure times are strictly shorter than their
    // respective frame periods (the muscle frame period is sync_ratio behavior
    // periods). Parameters that fail this check are rejected into the error
    // state.
    bool check_params_timing(const TriggerParams &params) const;

    // --- Pause switch and status output ----------------------------------
    void poll_pause_switch();
    // Latch the error state: drop every output, log `reason` over serial, and
    // refresh the status outputs. Cleared by the next valid
    // STREAM/START_RECORDING.
    void enter_error(const char *reason);
    bool is_active() const;
    StatusDisplay::Status current_status() const;
    // Push current_status() (and, when configured, the parameter lines) to both
    // the OLED panel and the RGB status LED.
    void refresh_status();

    DeviceIO &device_;
    SerialIO serial_;
    StatusDisplay display_;
    StatusLed status_led_;

    Mode mode_ = Mode::streaming;
    // True once parameters have been applied. begin() applies the default
    // streaming parameters, so the controller is configured (and streaming)
    // from startup; the "initializing" status is therefore only ever transient.
    bool configured_ = false;
    bool paused_ = false; // physical on/off switch override
    bool error_ = false;  // latched fault; halts triggering until reconfigured

    // Effective parameters and the timing values derived from them.
    TriggerParams params_;
    TriggerParams revert_to_params_;
    bool muscle_enabled_ = true; // params_.enable_muscle (active mode)
    unsigned long beh_period_us_ = 1000000; // behavior frame period (us)
    unsigned int sync_ratio_ = 1;           // behavior frames per muscle frame
    unsigned int beh_exp_us_ = 0;           // behavior exposure (us)
    unsigned int musc_exp_us_ = 0;          // blue LED on-time (us)

    // Scheduled-recording operation sequence (empty => open recording).
    std::deque<OperationStep> op_sequence_;

    // Timing-loop state.
    bool awaiting_musc_edge_ = true;   // waiting for the next common-time onset
    bool prev_common_time_ = false;    // previous is_musc_common_time() reading
    unsigned long group_start_us_ = 0; // micros() at the current group's onset
    unsigned int frame_in_group_ = 0;  // next behavior frame index in the group
    unsigned long next_beh_frame_us_ =
        0;                          // free-running mode: next frame due time
    bool beh_frame_active_ = false; // behavior shutter currently open
    unsigned long beh_frame_start_us_ = 0;
    bool musc_led_active_ = false; // blue excitation LED currently on
    unsigned long musc_led_start_us_ = 0;
    unsigned long beh_frame_count_ = 0; // behavior frames since recording start
};
