#include "trigger_firmware/status_led.h"

#include <Arduino.h>

#include "trigger_firmware/config.h"

void StatusLed::begin() {
    pinMode(config::statusLedRedPin, OUTPUT);
    pinMode(config::statusLedGreenPin, OUTPUT);
    pinMode(config::statusLedBluePin, OUTPUT);

    // Start dark, matching the pre-configuration "initializing" state.
    setStatus(StatusDisplay::Status::initializing);
}

void StatusLed::setStatus(StatusDisplay::Status status) {
    bool red = false;
    bool green = false;
    bool blue = false;
    switch (status) {
    case StatusDisplay::Status::initializing:
    case StatusDisplay::Status::paused:
        break; // off
    case StatusDisplay::Status::streaming:
        green = true;
        break;
    case StatusDisplay::Status::openRecording:
        blue = true;
        break;
    case StatusDisplay::Status::scheduledRecording:
        red = true;
        blue = true;
        break;
    case StatusDisplay::Status::error:
        red = true;
        break;
    }
    digitalWrite(config::statusLedRedPin, red ? HIGH : LOW);
    digitalWrite(config::statusLedGreenPin, green ? HIGH : LOW);
    digitalWrite(config::statusLedBluePin, blue ? HIGH : LOW);
}
