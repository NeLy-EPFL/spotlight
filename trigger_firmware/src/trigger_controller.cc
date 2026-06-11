#include "trigger_firmware/trigger_controller.h"

#include <optional>
#include <string>

#include <Arduino.h>
#include <esp_system.h> // esp_restart()

#include "trigger_firmware/config.h"

TriggerController::TriggerController() : device_(DeviceIO::get_instance()) {}

void TriggerController::begin() {
    serial_.begin();

    // DeviceIO configured its pins and drove every output to a safe state when
    // its singleton was constructed (via the device_ reference above);
    // re-assert that safe state here in case begin() is ever called after a
    // soft reset.
    device_.reset();

    // The physical on/off switch grounds the pin when engaged, so a LOW reading
    // means "paused". Sample it once so the startup status is correct.
    pinMode(config::on_off_switch_pin, INPUT_PULLUP);
    paused_ = digitalRead(config::on_off_switch_pin) == LOW;

    // Draws the initial "INITIALIZING" screen. A missing panel (begin() ==
    // false) is non-fatal: triggering does not depend on the display.
    display_.begin();

    // Light the status LED to match the initial "INITIALIZING" state (yellow).
    status_led_.begin();

    // Stream immediately with the default parameters so the behavior camera is
    // running before the host sends its first STREAM command. This overrides
    // the "INITIALIZING" status drawn above with "STREAMING".
    TriggerParams default_params;
    default_params.enable_muscle = config::default_enable_muscle;
    default_params.beh_frame_rate = config::default_beh_frame_rate;
    default_params.beh_exp_time = config::default_beh_exp_time;
    start_streaming(default_params);
}

void TriggerController::update() {
    poll_pause_switch();

    std::optional<std::string> line = serial_.update();
    if (line) {
        Command cmd = Command::parse(*line);
        if (cmd.is_valid) {
            handle_command(cmd);
        } else {
            enter_error("ignored malformed command");
        }
    }

    if (is_active()) {
        run_triggers(micros());
        // run_triggers() may latch the error state (e.g. on overrun); re-check
        // before stepping the operation sequence.
        if (is_active() && mode_ == Mode::scheduled_recording) {
            process_op_sequence();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Command handling                                                           */
/* -------------------------------------------------------------------------- */

void TriggerController::handle_command(const Command &cmd) {
    switch (cmd.cmd_type) {
    case CmdType::stream:
        handle_stream(cmd);
        break;
    case CmdType::start_recording:
        handle_start_recording(cmd);
        break;
    case CmdType::stop_recording:
        handle_stop_recording();
        break;
    case CmdType::log:
        handle_log(cmd);
        break;
    case CmdType::reset:
        handle_reset();
        break;
    }
}

void TriggerController::handle_stream(const Command &cmd) {
    if (!check_params_timing(cmd.params)) {
        enter_error("STREAM rejected: camera exposure not shorter than frame "
                    "period");
        return;
    }
    start_streaming(cmd.params);
}

void TriggerController::start_streaming(const TriggerParams &params) {
    error_ = false;
    apply_params(params);
    op_sequence_.clear();
    device_.turn_off_opto_ch(OptoChannel::all);
    mode_ = Mode::streaming;
    configured_ = true;
    reset_timing();
    refresh_status();
}

void TriggerController::handle_start_recording(const Command &cmd) {
    // Both the recording and the revert-to parameters must be timing-valid:
    // the latter is applied unchecked when the recording ends.
    if (!check_params_timing(cmd.rec_params) ||
        !check_params_timing(cmd.revert_to_params)) {
        enter_error(
            "START_RECORDING rejected: camera exposure not shorter than "
            "frame period");
        return;
    }
    error_ = false;
    apply_params(cmd.rec_params);
    revert_to_params_ = cmd.revert_to_params;
    op_sequence_ = cmd.op_sequence;
    configured_ = true;

    // Let frames acquired with the previous parameters drain out of the camera
    // buffers before the recorded session begins (see cam_flush_time_us).
    device_.reset();
    delayMicroseconds(cam_flush_time_us);

    mode_ =
        op_sequence_.empty() ? Mode::open_recording : Mode::scheduled_recording;
    reset_timing();
    refresh_status();
}

void TriggerController::handle_stop_recording() {
    if (mode_ == Mode::open_recording) {
        revert_to_streaming();
        return;
    }
    // A scheduled recording ends via its op_sequence STOP step, never via an
    // external STOP_RECORDING; one arriving mid-schedule is a protocol error.
    if (mode_ == Mode::scheduled_recording) {
        enter_error("STOP_RECORDING received during a scheduled recording");
        return;
    }
    // Otherwise there is no recording to end (streaming, or before any
    // START_RECORDING). This is harmless, so log a warning and ignore it rather
    // than faulting.
    Serial.println("Triggering controller warning: ignoring STOP_RECORDING "
                   "received with no open recording");
}

void TriggerController::handle_log(const Command &cmd) {
    Serial.print("Triggering controller received log message: ");
    Serial.println(cmd.log_msg.c_str());
}

void TriggerController::handle_reset() {
    // Software reset requested by the host: reboot the MCU so it comes back
    // from a clean, known state, exactly as if the physical reset button were
    // pressed. Drop every output first, then show "RESETTING" (with all
    // parameter lines blank) on the OLED and status LED so the reset is
    // visible. esp_restart() does not return; on the next boot begin()
    // re-initializes everything and resumes default streaming.
    device_.reset();
    display_.clear();
    display_.set_status(StatusDisplay::Status::resetting);
    display_.render();
    status_led_.set_status(StatusDisplay::Status::resetting);

    Serial.println("Triggering controller resetting (esp_restart)");
    Serial.flush(); // let the message leave the USB TX buffer before rebooting

    esp_restart();
}

/* -------------------------------------------------------------------------- */
/* Timing engine                                                              */
/* -------------------------------------------------------------------------- */

void TriggerController::run_triggers(unsigned long now_us) {
    if (muscle_enabled_) {
        run_muscle_synced_triggers(now_us);
    } else {
        run_free_running_triggers(now_us);
    }
}

void TriggerController::run_muscle_synced_triggers(unsigned long now_us) {
    // Detect the onset of the muscle camera's common time (signal HIGH). Each
    // onset starts a new sync group whose first behavior frame is locked to it.
    bool common = device_.is_musc_common_time();
    bool onset = common && !prev_common_time_;
    prev_common_time_ = common;

    // A fresh common-time onset while still mid-group means the muscle camera
    // started its next frame before this group's behavior frames finished: the
    // configured behavior frame rate cannot keep up with the muscle camera.
    if (onset && !awaiting_musc_edge_) {
        enter_error(
            "muscle-frame overrun: behavior frames did not finish before "
            "the next muscle frame");
        return;
    }

    if (awaiting_musc_edge_ && onset) {
        group_start_us_ = now_us;
        frame_in_group_ = 0;
        awaiting_musc_edge_ = false;
    }

    // Start the next behavior frame of the group once it is due. Frame i of a
    // group is scheduled at group_start + i * beh_period on the controller
    // clock.
    if (!awaiting_musc_edge_ && !beh_frame_active_ &&
        frame_in_group_ < sync_ratio_ &&
        (now_us - group_start_us_) >=
            static_cast<unsigned long>(frame_in_group_) * beh_period_us_) {
        device_.start_beh_cam_trigger();
        device_.turn_on_beh_led();
        beh_frame_active_ = true;
        beh_frame_start_us_ = now_us;

        // The 0th frame of every group coincides with the muscle common time,
        // so the blue excitation LED is pulsed only then.
        if (frame_in_group_ == 0) {
            device_.turn_on_musc_led();
            musc_led_active_ = true;
            musc_led_start_us_ = now_us;
        }

        ++frame_in_group_;
        ++beh_frame_count_;
        if (frame_in_group_ >= sync_ratio_) {
            awaiting_musc_edge_ = true; // group complete; await the next onset
        }
    }

    // Close the behavior shutter (and IR LED) after the exposure time.
    if (beh_frame_active_ && (now_us - beh_frame_start_us_) >= beh_exp_us_) {
        device_.stop_beh_cam_trigger();
        device_.turn_off_beh_led();
        beh_frame_active_ = false;
    }

    // Turn the blue excitation LED off after the muscle effective exposure.
    if (musc_led_active_ && (now_us - musc_led_start_us_) >= musc_exp_us_) {
        device_.turn_off_musc_led();
        musc_led_active_ = false;
    }
}

void TriggerController::run_free_running_triggers(unsigned long now_us) {
    // Free-running mode: the muscle camera is ignored, so there is no
    // common-time signal, no sync group, and no blue excitation LED. Behavior
    // frames are scheduled purely on the controller's own clock at
    // beh_period_us_.

    // Start the next behavior frame once it is due. next_beh_frame_us_ holds
    // the scheduled micros() time of the next frame and is advanced by one
    // period as each frame starts; the signed-difference test is
    // wraparound-safe.
    if (!beh_frame_active_ &&
        static_cast<long>(now_us - next_beh_frame_us_) >= 0) {
        device_.start_beh_cam_trigger();
        device_.turn_on_beh_led();
        beh_frame_active_ = true;
        beh_frame_start_us_ = now_us;
        next_beh_frame_us_ += beh_period_us_;
        ++beh_frame_count_;
    }

    // Close the behavior shutter (and IR LED) after the exposure time.
    // beh_exp_us_ is guaranteed shorter than beh_period_us_, so the frame
    // always closes before the next one is due.
    if (beh_frame_active_ && (now_us - beh_frame_start_us_) >= beh_exp_us_) {
        device_.stop_beh_cam_trigger();
        device_.turn_off_beh_led();
        beh_frame_active_ = false;
    }
}

void TriggerController::process_op_sequence() {
    while (!op_sequence_.empty()) {
        const OperationStep &step = op_sequence_.front();
        if (step.frame_idx > beh_frame_count_) {
            break; // this step's frame has not been reached yet
        }

        if (step.op == OpType::stop) {
            // Let the frame in progress finish before reverting, so the last
            // recorded frame is complete.
            if (beh_frame_active_) {
                break;
            }
            revert_to_streaming();
            return; // mode and op_sequence_ have changed
        }

        if (step.op == OpType::on) {
            device_.turn_on_opto_ch(step.channel);
        } else {
            device_.turn_off_opto_ch(step.channel);
        }
        op_sequence_.pop_front();
    }
}

void TriggerController::revert_to_streaming() {
    error_ = false; // reverting establishes a known-good streaming state
    apply_params(revert_to_params_);
    op_sequence_.clear();
    device_.turn_off_opto_ch(OptoChannel::all);
    mode_ = Mode::streaming;
    reset_timing();
    refresh_status();
}

void TriggerController::reset_timing() {
    awaiting_musc_edge_ = true;
    // Seed the edge detector with the current level so a common time already in
    // progress does not count as a fresh onset.
    prev_common_time_ = device_.is_musc_common_time();
    frame_in_group_ = 0;
    group_start_us_ = micros();
    // Free-running mode: schedule the first behavior frame immediately.
    next_beh_frame_us_ = group_start_us_;
    beh_frame_active_ = false;
    musc_led_active_ = false;
    beh_frame_count_ = 0;

    // Drop any output left mid-pulse to a known state.
    device_.stop_beh_cam_trigger();
    device_.turn_off_beh_led();
    device_.turn_off_musc_led();
}

void TriggerController::apply_params(const TriggerParams &params) {
    params_ = params;
    muscle_enabled_ = params_.enable_muscle; // selects the timing path
    beh_period_us_ = 1000000UL / params_.beh_frame_rate; // beh_frame_rate >= 1
    sync_ratio_ = params_.beh_musc_sync_ratio;           // >= 1
    beh_exp_us_ = params_.beh_exp_time;
    musc_exp_us_ = params_.musc_eff_exp_time;
}

bool TriggerController::check_params_timing(const TriggerParams &params) const {
    // beh_frame_rate and beh_musc_sync_ratio are guaranteed >= 1 by the
    // protocol.
    unsigned long beh_period_us = 1000000UL / params.beh_frame_rate;
    if (params.beh_exp_time >= beh_period_us) {
        return false;
    }
    // In free-running mode the muscle camera is unused, so only the behavior
    // timing is constrained.
    if (!params.enable_muscle) {
        return true;
    }
    // One muscle frame spans a whole sync group of behavior frames.
    unsigned long musc_period_us =
        beh_period_us * static_cast<unsigned long>(params.beh_musc_sync_ratio);
    return params.musc_eff_exp_time < musc_period_us;
}

/* -------------------------------------------------------------------------- */
/* Pause switch and display                                                   */
/* -------------------------------------------------------------------------- */

void TriggerController::poll_pause_switch() {
    bool paused_now = digitalRead(config::on_off_switch_pin) == LOW;
    if (paused_now == paused_) {
        return;
    }
    paused_ = paused_now;
    if (paused_) {
        device_.reset(); // drop every output while paused
    } else {
        reset_timing(); // resume cleanly on the next common-time onset
    }
    refresh_status();
}

void TriggerController::enter_error(const char *reason) {
    error_ = true;
    device_.reset(); // drop every output; stay in error until reconfigured
    Serial.print("Triggering controller error: ");
    Serial.println(reason);
    refresh_status();
}

bool TriggerController::is_active() const {
    return configured_ && !paused_ && !error_;
}

StatusDisplay::Status TriggerController::current_status() const {
    if (error_) {
        return StatusDisplay::Status::error;
    }
    if (!configured_) {
        return StatusDisplay::Status::initializing;
    }
    if (paused_) {
        return StatusDisplay::Status::paused;
    }
    switch (mode_) {
    case Mode::streaming:
        return StatusDisplay::Status::streaming;
    case Mode::open_recording:
        return StatusDisplay::Status::open_recording;
    case Mode::scheduled_recording:
        return StatusDisplay::Status::scheduled_recording;
    }
    return StatusDisplay::Status::error;
}

void TriggerController::refresh_status() {
    StatusDisplay::Status status = current_status();
    display_.set_status(status);
    if (configured_) {
        display_.set_beh_frame_rate(params_.beh_frame_rate);
        display_.set_beh_exp_time(params_.beh_exp_time);
        // The sync ratio and muscle exposure are meaningless when muscle
        // imaging is disabled (the behavior camera free-runs), so show
        // "N/A"/"OFF".
        if (params_.enable_muscle) {
            display_.set_beh_musc_sync_ratio(params_.beh_musc_sync_ratio);
            display_.set_musc_exp_time(params_.musc_eff_exp_time);
        } else {
            display_.set_beh_musc_ratio_na();
            display_.set_musc_exp_off();
        }
    }
    display_.render();
    status_led_.set_status(status);
}
