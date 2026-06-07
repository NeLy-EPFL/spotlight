#include "trigger_firmware/status_led.h"

#include <Arduino.h>

#include "trigger_firmware/config.h"

void StatusLed::begin() {
    pinMode(config::statusLedRedPin, OUTPUT);
    pinMode(config::statusLedGreenPin, OUTPUT);
    pinMode(config::statusLedBluePin, OUTPUT);

    // Show the initial "initializing" status (yellow) until the first RUN
    // command reconfigures the controller.
    setStatus(StatusDisplay::Status::initializing);
}

void StatusLed::setStatus(StatusDisplay::Status status) {
    Color color = colorFor(status);
    analogWrite(config::statusLedRedPin, color.red);
    analogWrite(config::statusLedGreenPin, color.green);
    analogWrite(config::statusLedBluePin, color.blue);
}
