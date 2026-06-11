#include "trigger_firmware/status_led.h"

#include <Arduino.h>

#include "trigger_firmware/config.h"

void StatusLed::begin() {
    pinMode(config::status_led_red_pin, OUTPUT);
    pinMode(config::status_led_green_pin, OUTPUT);
    pinMode(config::status_led_blue_pin, OUTPUT);

    // Show the initial "initializing" status (yellow) until the first RUN
    // command reconfigures the controller.
    set_status(StatusDisplay::Status::initializing);
}

void StatusLed::set_status(StatusDisplay::Status status) {
    Color color = color_for(status);
    analogWrite(config::status_led_red_pin, color.red);
    analogWrite(config::status_led_green_pin, color.green);
    analogWrite(config::status_led_blue_pin, color.blue);
}
