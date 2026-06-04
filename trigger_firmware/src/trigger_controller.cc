#include "trigger_firmware/trigger_controller.h"

#include <optional>
#include <string>

#include <Arduino.h>

#include "trigger_firmware/config.h"

TriggerController::TriggerController() : device_(DeviceIO::getInstance()) {}

void TriggerController::begin() {
    serial_.begin();

    // DeviceIO configured its pins and drove every output to a safe state when
    // its singleton was constructed (via the device_ reference above);
    // re-assert that safe state here in case begin() is ever called after a
    // soft reset.
    device_.reset();

    // The physical on/off switch grounds the pin when engaged, so a LOW reading
    // means "paused". Sample it once so the startup status is correct.
    pinMode(config::onOffSwitchPin, INPUT_PULLUP);
    paused_ = digitalRead(config::onOffSwitchPin) == LOW;

    // Draws the initial "INITIALIZING" screen. A missing panel (begin() ==
    // false) is non-fatal: triggering does not depend on the display.
    display_.begin();

    // Light the status LED to match the initial "INITIALIZING" state (off).
    statusLed_.begin();
}

void TriggerController::update() {
    pollPauseSwitch();

    std::optional<std::string> line = serial_.update();
    if (line) {
        Command cmd = Command::parse(*line);
        if (cmd.isValid) {
            handleCommand(cmd);
        } else {
            enterError("ignored malformed command");
        }
    }

    if (isActive()) {
        runTriggers(micros());
        // runTriggers() may latch the error state (e.g. on overrun); re-check
        // before stepping the operation sequence.
        if (isActive() && mode_ == Mode::scheduledRecording) {
            processOpSequence();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Command handling                                                           */
/* -------------------------------------------------------------------------- */

void TriggerController::handleCommand(const Command &cmd) {
    switch (cmd.cmdType) {
    case CmdType::STREAM:
        handleStream(cmd);
        break;
    case CmdType::START_RECORDING:
        handleStartRecording(cmd);
        break;
    case CmdType::STOP_RECORDING:
        handleStopRecording();
        break;
    case CmdType::LOG:
        handleLog(cmd);
        break;
    }
}

void TriggerController::handleStream(const Command &cmd) {
    if (!checkParamsTiming(cmd.params)) {
        enterError(
            "STREAM rejected: camera exposure not shorter than frame "
            "period");
        return;
    }
    error_ = false;
    applyParams(cmd.params);
    opSequence_.clear();
    device_.turnOffOptoCh(OptoChannel::ALL);
    mode_ = Mode::streaming;
    configured_ = true;
    resetTiming();
    refreshStatus();
}

void TriggerController::handleStartRecording(const Command &cmd) {
    // Both the recording and the revert-to parameters must be timing-valid:
    // the latter is applied unchecked when the recording ends.
    if (!checkParamsTiming(cmd.recParams) ||
        !checkParamsTiming(cmd.revertToParams)) {
        enterError(
            "START_RECORDING rejected: camera exposure not shorter than "
            "frame period");
        return;
    }
    error_ = false;
    applyParams(cmd.recParams);
    revertToParams_ = cmd.revertToParams;
    opSequence_ = cmd.opSequence;
    configured_ = true;

    // Let frames acquired with the previous parameters drain out of the camera
    // buffers before the recorded session begins (see camFlushTimeUs).
    device_.reset();
    delayMicroseconds(camFlushTimeUs);

    mode_ =
        opSequence_.empty() ? Mode::openRecording : Mode::scheduledRecording;
    resetTiming();
    refreshStatus();
}

void TriggerController::handleStopRecording() {
    // STOP_RECORDING only ends an open recording (a scheduled recording ends
    // via its opSequence STOP step). Anything else - including a STOP_RECORDING
    // before any START_RECORDING - is a protocol error.
    if (mode_ != Mode::openRecording) {
        enterError("STOP_RECORDING received outside an open recording");
        return;
    }
    revertToStreaming();
}

void TriggerController::handleLog(const Command &cmd) {
    Serial.print("Triggering controller received log message: ");
    Serial.println(cmd.logMsg.c_str());
}

/* -------------------------------------------------------------------------- */
/* Timing engine                                                              */
/* -------------------------------------------------------------------------- */

void TriggerController::runTriggers(unsigned long nowUs) {
    // Detect the onset of the muscle camera's common time (signal LOW). Each
    // onset starts a new sync group whose first behavior frame is locked to it.
    bool common = device_.isMuscCommonTime();
    bool onset = common && !prevCommonTime_;
    prevCommonTime_ = common;

    // A fresh common-time onset while still mid-group means the muscle camera
    // started its next frame before this group's behavior frames finished: the
    // configured behavior frame rate cannot keep up with the muscle camera.
    if (onset && !awaitingMuscEdge_) {
        enterError(
            "muscle-frame overrun: behavior frames did not finish before "
            "the next muscle frame");
        return;
    }

    if (awaitingMuscEdge_ && onset) {
        groupStartUs_ = nowUs;
        frameInGroup_ = 0;
        awaitingMuscEdge_ = false;
    }

    // Start the next behavior frame of the group once it is due. Frame i of a
    // group is scheduled at groupStart + i * behPeriod on the controller clock.
    if (!awaitingMuscEdge_ && !behFrameActive_ && frameInGroup_ < syncRatio_ &&
        (nowUs - groupStartUs_) >=
            static_cast<unsigned long>(frameInGroup_) * behPeriodUs_) {
        device_.startBehCamTrigger();
        device_.turnOnBehLED();
        behFrameActive_ = true;
        behFrameStartUs_ = nowUs;

        // The 0th frame of every group coincides with the muscle common time,
        // so the blue excitation LED is pulsed only then.
        if (frameInGroup_ == 0) {
            device_.turnOnMuscLED();
            muscLEDActive_ = true;
            muscLEDStartUs_ = nowUs;
        }

        ++frameInGroup_;
        ++behFrameCount_;
        if (frameInGroup_ >= syncRatio_) {
            awaitingMuscEdge_ = true; // group complete; await the next onset
        }
    }

    // Close the behavior shutter (and IR LED) after the exposure time.
    if (behFrameActive_ && (nowUs - behFrameStartUs_) >= behExpUs_) {
        device_.stopBehCamTrigger();
        device_.turnOffBehLED();
        behFrameActive_ = false;
    }

    // Turn the blue excitation LED off after the muscle effective exposure.
    if (muscLEDActive_ && (nowUs - muscLEDStartUs_) >= muscExpUs_) {
        device_.turnOffMuscLED();
        muscLEDActive_ = false;
    }
}

void TriggerController::processOpSequence() {
    while (!opSequence_.empty()) {
        const OperationStep &step = opSequence_.front();
        if (step.frameIdx > behFrameCount_) {
            break; // this step's frame has not been reached yet
        }

        if (step.op == OpType::STOP) {
            // Let the frame in progress finish before reverting, so the last
            // recorded frame is complete.
            if (behFrameActive_) {
                break;
            }
            revertToStreaming();
            return; // mode and opSequence_ have changed
        }

        if (step.op == OpType::ON) {
            device_.turnOnOptoCh(step.channel);
        } else {
            device_.turnOffOptoCh(step.channel);
        }
        opSequence_.pop_front();
    }
}

void TriggerController::revertToStreaming() {
    error_ = false; // reverting establishes a known-good streaming state
    applyParams(revertToParams_);
    opSequence_.clear();
    device_.turnOffOptoCh(OptoChannel::ALL);
    mode_ = Mode::streaming;
    resetTiming();
    refreshStatus();
}

void TriggerController::resetTiming() {
    awaitingMuscEdge_ = true;
    // Seed the edge detector with the current level so a common time already in
    // progress does not count as a fresh onset.
    prevCommonTime_ = device_.isMuscCommonTime();
    frameInGroup_ = 0;
    groupStartUs_ = micros();
    behFrameActive_ = false;
    muscLEDActive_ = false;
    behFrameCount_ = 0;

    // Drop any output left mid-pulse to a known state.
    device_.stopBehCamTrigger();
    device_.turnOffBehLED();
    device_.turnOffMuscLED();
}

void TriggerController::applyParams(const TriggerParams &params) {
    params_ = params;
    behPeriodUs_ = 1000000UL / params_.behFrameRate; // behFrameRate >= 1
    syncRatio_ = params_.behMuscSyncRatio;           // >= 1
    behExpUs_ = params_.behExpTime;
    muscExpUs_ = params_.muscEffExpTime;
}

bool TriggerController::checkParamsTiming(const TriggerParams &params) const {
    // behFrameRate and behMuscSyncRatio are guaranteed >= 1 by the protocol.
    unsigned long behPeriodUs = 1000000UL / params.behFrameRate;
    // One muscle frame spans a whole sync group of behavior frames.
    unsigned long muscPeriodUs =
        behPeriodUs * static_cast<unsigned long>(params.behMuscSyncRatio);
    return params.behExpTime < behPeriodUs &&
           params.muscEffExpTime < muscPeriodUs;
}

/* -------------------------------------------------------------------------- */
/* Pause switch and display                                                   */
/* -------------------------------------------------------------------------- */

void TriggerController::pollPauseSwitch() {
    bool pausedNow = digitalRead(config::onOffSwitchPin) == LOW;
    if (pausedNow == paused_) {
        return;
    }
    paused_ = pausedNow;
    if (paused_) {
        device_.reset(); // drop every output while paused
    } else {
        resetTiming(); // resume cleanly on the next common-time onset
    }
    refreshStatus();
}

void TriggerController::enterError(const char *reason) {
    error_ = true;
    device_.reset(); // drop every output; stay in error until reconfigured
    Serial.print("Triggering controller error: ");
    Serial.println(reason);
    refreshStatus();
}

bool TriggerController::isActive() const {
    return configured_ && !paused_ && !error_;
}

StatusDisplay::Status TriggerController::currentStatus() const {
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
    case Mode::openRecording:
        return StatusDisplay::Status::openRecording;
    case Mode::scheduledRecording:
        return StatusDisplay::Status::scheduledRecording;
    }
    return StatusDisplay::Status::error;
}

void TriggerController::refreshStatus() {
    StatusDisplay::Status status = currentStatus();
    display_.setStatus(status);
    if (configured_) {
        display_.setBehFrameRate(params_.behFrameRate);
        display_.setBehMuscSyncRatio(params_.behMuscSyncRatio);
        display_.setBehExpTime(params_.behExpTime);
        display_.setMuscExpTime(params_.muscEffExpTime);
    }
    display_.render();
    statusLed_.setStatus(status);
}
