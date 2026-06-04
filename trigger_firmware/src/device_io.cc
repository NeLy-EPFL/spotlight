#include "trigger_firmware/device_io.h"
#include "trigger_firmware/config.h"

#include <Arduino.h>

DeviceIO &DeviceIO::getInstance() {
    static DeviceIO instance;
    return instance;
}

DeviceIO::DeviceIO() {
    // Configure every owned pin before reset() drives it. The muscle camera
    // status line is the only input (its level reports the common time).
    pinMode(config::behCamPin, OUTPUT);
    pinMode(config::irLEDPin, OUTPUT);
    pinMode(config::muscCamTriggerPin, OUTPUT);
    pinMode(config::muscCamStatusPin, INPUT);
    pinMode(config::blueLEDPin, OUTPUT);
    pinMode(config::optoCh2Pin, OUTPUT);
    pinMode(config::optoCh3Pin, OUTPUT);

    reset();
}

void DeviceIO::reset() {
    isBehCamTriggerOn_ = true;
    stopBehCamTrigger();

    isBehLEDOn_ = true;
    turnOffBehLED();

    isMuscCamTriggerOn_ = true;
    stopMuscCamTrigger();

    isMuscLEDOn_ = true;
    turnOffMuscLED();

    isOptoCh2On_ = true;
    isOptoCh3On_ = true;
    turnOffOptoCh(OptoChannel::ALL);
}

void DeviceIO::startBehCamTrigger() {
    if (!isBehCamTriggerOn_) {
        digitalWrite(config::behCamPin, HIGH);
        isBehCamTriggerOn_ = true;
    }
}

void DeviceIO::stopBehCamTrigger() {
    if (isBehCamTriggerOn_) {
        digitalWrite(config::behCamPin, LOW);
        isBehCamTriggerOn_ = false;
    }
}

void DeviceIO::turnOnBehLED() {
    if (!isBehLEDOn_) {
        digitalWrite(config::irLEDPin, HIGH);
        isBehLEDOn_ = true;
    }
}

void DeviceIO::turnOffBehLED() {
    if (isBehLEDOn_) {
        digitalWrite(config::irLEDPin, LOW);
        isBehLEDOn_ = false;
    }
}

void DeviceIO::startMuscCamTrigger() {
    if (!isMuscCamTriggerOn_) {
        digitalWrite(config::muscCamTriggerPin, HIGH);
        isMuscCamTriggerOn_ = true;
    }
}

void DeviceIO::stopMuscCamTrigger() {
    if (isMuscCamTriggerOn_) {
        digitalWrite(config::muscCamTriggerPin, LOW);
        isMuscCamTriggerOn_ = false;
    }
}

bool DeviceIO::isMuscCommonTime() {
    return digitalRead(config::muscCamStatusPin) == LOW;
}

void DeviceIO::turnOnMuscLED() {
    if (!isMuscLEDOn_) {
        digitalWrite(config::blueLEDPin, HIGH);
        isMuscLEDOn_ = true;
    }
}

void DeviceIO::turnOffMuscLED() {
    if (isMuscLEDOn_) {
        digitalWrite(config::blueLEDPin, LOW);
        isMuscLEDOn_ = false;
    }
}

bool DeviceIO::turnOnOptoCh(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::CH2:
        if (!isOptoCh2On_) {
            digitalWrite(config::optoCh2Pin, HIGH);
            isOptoCh2On_ = true;
        }
        return true;
    case OptoChannel::CH3:
        if (!isOptoCh3On_) {
            digitalWrite(config::optoCh3Pin, HIGH);
            isOptoCh3On_ = true;
        }
        return true;
    case OptoChannel::ALL:
        turnOnOptoCh(OptoChannel::CH2);
        turnOnOptoCh(OptoChannel::CH3);
        return true;
    }
    return false;
}

bool DeviceIO::turnOffOptoCh(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::CH2:
        if (isOptoCh2On_) {
            digitalWrite(config::optoCh2Pin, LOW);
            isOptoCh2On_ = false;
        }
        return true;
    case OptoChannel::CH3:
        if (isOptoCh3On_) {
            digitalWrite(config::optoCh3Pin, LOW);
            isOptoCh3On_ = false;
        }
        return true;
    case OptoChannel::ALL:
        turnOffOptoCh(OptoChannel::CH2);
        turnOffOptoCh(OptoChannel::CH3);
        return true;
    }
    return false;
}

bool DeviceIO::isOptoChOn(OptoChannel channel) {
    switch (channel) {
    case OptoChannel::CH2:
        return isOptoCh2On_;
    case OptoChannel::CH3:
        return isOptoCh3On_;
    case OptoChannel::ALL:
        return isOptoCh2On_ && isOptoCh3On_;
    }
    return false;
}
