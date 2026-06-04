#pragma once

#include <comm_protocol/protocol.h>

/**
 * Tracks and drives the state of every output the trigger controller owns: the
 * behavior and muscle camera triggers, the IR and blue excitation LEDs, and the
 * optogenetics channels. Each setter is idempotent: it only touches the
 * corresponding pin when the cached state actually changes.
 *
 * There is a single set of physical outputs, so this is a singleton: access the
 * sole instance through DeviceIO::getInstance().
 */
class DeviceIO {
  public:
    static DeviceIO &getInstance();

    DeviceIO(const DeviceIO &) = delete;
    DeviceIO &operator=(const DeviceIO &) = delete;

    // Behavior camera trigger and IR illumination LED.
    void startBehCamTrigger();
    void stopBehCamTrigger();
    void turnOnBehLED();
    void turnOffBehLED();

    // Muscle camera trigger and blue excitation LED.
    void startMuscCamTrigger();
    void stopMuscCamTrigger();
    bool isMuscCommonTime();
    void turnOnMuscLED();
    void turnOffMuscLED();

    // Optogenetics channels. `channel` is CH2 or CH3 for a single channel, or
    // ALL to act on both at once. Returns false for an unknown channel.
    bool turnOnOptoCh(OptoChannel channel);
    bool turnOffOptoCh(OptoChannel channel);
    bool isOptoChOn(OptoChannel channel);

    // Drive every output to its safe default (triggers stopped, LEDs and opto
    // channels off).
    void reset();

  private:
    DeviceIO();

    bool isBehCamTriggerOn_;
    bool isBehLEDOn_;
    bool isMuscCamTriggerOn_;
    bool isMuscLEDOn_;
    bool isOptoCh2On_;
    bool isOptoCh3On_;
};
