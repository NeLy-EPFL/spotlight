#include "trigger_firmware/device_io.h"
#include "trigger_firmware/config.h"

#include <Arduino.h>

DeviceIO &DeviceIO::get_instance() {
    static DeviceIO instance;
    return instance;
}

DeviceIO::DeviceIO() {
    // Configure every owned pin before reset() drives it. The muscle camera
    // status line is the only input (its level reports the common time). It is
    // read with an internal pull-down so that when the PCO Status Expos output
    // is undriven (camera off/booting, output disabled, or cable unplugged) the
    // line reads LOW ("no common time"): the controller simply waits rather
    // than seeing spurious onsets. The PCO output is push-pull 3.3 V LVTTL when
    // active, so the weak pull-down does not fight it.
    pinMode(config::beh_cam_pin, OUTPUT);
    pinMode(config::ir_led_pin, OUTPUT);
    pinMode(config::musc_cam_trigger_pin, OUTPUT);
    pinMode(config::musc_cam_status_pin, INPUT_PULLDOWN);
    pinMode(config::blue_led_pin, OUTPUT);
    pinMode(config::opto_ch2_pin, OUTPUT);
    pinMode(config::opto_ch3_pin, OUTPUT);

    reset();
}

void DeviceIO::reset() {
    is_beh_cam_trigger_on_ = true;
    stop_beh_cam_trigger();

    is_beh_led_on_ = true;
    turn_off_beh_led();

    is_musc_cam_trigger_on_ = true;
    stop_musc_cam_trigger();

    is_musc_led_on_ = true;
    turn_off_musc_led();

    is_opto_ch2_on_ = true;
    is_opto_ch3_on_ = true;
    turn_off_opto_ch(OptoChannel::all);
}

void DeviceIO::start_beh_cam_trigger() {
    if (!is_beh_cam_trigger_on_) {
        digitalWrite(config::beh_cam_pin, HIGH);
        is_beh_cam_trigger_on_ = true;
    }
}

void DeviceIO::stop_beh_cam_trigger() {
    if (is_beh_cam_trigger_on_) {
        digitalWrite(config::beh_cam_pin, LOW);
        is_beh_cam_trigger_on_ = false;
    }
}

void DeviceIO::turn_on_beh_led() {
    if (!is_beh_led_on_) {
        digitalWrite(config::ir_led_pin, HIGH);
        is_beh_led_on_ = true;
    }
}

void DeviceIO::turn_off_beh_led() {
    if (is_beh_led_on_) {
        digitalWrite(config::ir_led_pin, LOW);
        is_beh_led_on_ = false;
    }
}

void DeviceIO::start_musc_cam_trigger() {
    if (!is_musc_cam_trigger_on_) {
        digitalWrite(config::musc_cam_trigger_pin, HIGH);
        is_musc_cam_trigger_on_ = true;
    }
}

void DeviceIO::stop_musc_cam_trigger() {
    if (is_musc_cam_trigger_on_) {
        digitalWrite(config::musc_cam_trigger_pin, LOW);
        is_musc_cam_trigger_on_ = false;
    }
}

bool DeviceIO::is_musc_common_time() {
    return digitalRead(config::musc_cam_status_pin) == HIGH;
}

void DeviceIO::turn_on_musc_led() {
    if (!is_musc_led_on_) {
        digitalWrite(config::blue_led_pin, HIGH);
        is_musc_led_on_ = true;
    }
}

void DeviceIO::turn_off_musc_led() {
    if (is_musc_led_on_) {
        digitalWrite(config::blue_led_pin, LOW);
        is_musc_led_on_ = false;
    }
}

bool DeviceIO::turn_on_opto_ch(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::ch2:
        if (!is_opto_ch2_on_) {
            digitalWrite(config::opto_ch2_pin, HIGH);
            is_opto_ch2_on_ = true;
        }
        return true;
    case OptoChannel::ch3:
        if (!is_opto_ch3_on_) {
            digitalWrite(config::opto_ch3_pin, HIGH);
            is_opto_ch3_on_ = true;
        }
        return true;
    case OptoChannel::all:
        turn_on_opto_ch(OptoChannel::ch2);
        turn_on_opto_ch(OptoChannel::ch3);
        return true;
    }
    return false;
}

bool DeviceIO::turn_off_opto_ch(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::ch2:
        if (is_opto_ch2_on_) {
            digitalWrite(config::opto_ch2_pin, LOW);
            is_opto_ch2_on_ = false;
        }
        return true;
    case OptoChannel::ch3:
        if (is_opto_ch3_on_) {
            digitalWrite(config::opto_ch3_pin, LOW);
            is_opto_ch3_on_ = false;
        }
        return true;
    case OptoChannel::all:
        turn_off_opto_ch(OptoChannel::ch2);
        turn_off_opto_ch(OptoChannel::ch3);
        return true;
    }
    return false;
}

bool DeviceIO::is_opto_ch_on(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::ch2:
        return is_opto_ch2_on_;
    case OptoChannel::ch3:
        return is_opto_ch3_on_;
    case OptoChannel::all:
        return is_opto_ch2_on_ && is_opto_ch3_on_;
    }
    return false;
}
