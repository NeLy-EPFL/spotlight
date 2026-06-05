// On-device AUnit tests for DeviceIO.
//
// DeviceIO owns the physical outputs, so these run on the actual Arduino Nano
// ESP32 (uploaded and executed by `pio test`, see the project README). They
// exercise the cached output-state bookkeeping that the rest of the firmware
// relies on: the idempotent setters, the OptoChannel::ALL aggregation, and
// reset(). They do not assert on electrical pin levels, only on the logical
// state DeviceIO reports back through isOptoChOn().

#include <Arduino.h>
#include <AUnit.h>

#include "trigger_firmware/device_io.h"

// The Arduino Nano ESP32's native USB CDC reports "connected" immediately and is
// not reset when the port is opened, so the board would otherwise print its
// results before the test host (`pio test`) attaches and they would be lost.
// Delaying the start of the run leaves time for the host to open the port first.
// See the firmware test instructions in the project README.
static constexpr unsigned long kTestHostConnectDelayMs = 20000;

// There is a single physical output set, so DeviceIO is a singleton. Each test
// starts from reset() to get a known state regardless of run order.

test(deviceIo_resetTurnsEverythingOff) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.turnOnOptoCh(OptoChannel::ALL);
    dev.reset();
    assertFalse(dev.isOptoChOn(OptoChannel::CH2));
    assertFalse(dev.isOptoChOn(OptoChannel::CH3));
    assertFalse(dev.isOptoChOn(OptoChannel::ALL));
}

test(deviceIo_turnOnSingleChannel) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.reset();

    assertTrue(dev.turnOnOptoCh(OptoChannel::CH2));
    assertTrue(dev.isOptoChOn(OptoChannel::CH2));
    assertFalse(dev.isOptoChOn(OptoChannel::CH3));
    // ALL is on only when both channels are on.
    assertFalse(dev.isOptoChOn(OptoChannel::ALL));
}

test(deviceIo_turnOnAllThenQueryParts) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.reset();

    assertTrue(dev.turnOnOptoCh(OptoChannel::ALL));
    assertTrue(dev.isOptoChOn(OptoChannel::CH2));
    assertTrue(dev.isOptoChOn(OptoChannel::CH3));
    assertTrue(dev.isOptoChOn(OptoChannel::ALL));
}

test(deviceIo_turnOffSingleChannel) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.reset();
    dev.turnOnOptoCh(OptoChannel::ALL);

    assertTrue(dev.turnOffOptoCh(OptoChannel::CH3));
    assertTrue(dev.isOptoChOn(OptoChannel::CH2));
    assertFalse(dev.isOptoChOn(OptoChannel::CH3));
    assertFalse(dev.isOptoChOn(OptoChannel::ALL));
}

test(deviceIo_settersAreIdempotent) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.reset();

    // Calling a setter repeatedly must leave the reported state unchanged.
    dev.turnOnOptoCh(OptoChannel::CH2);
    dev.turnOnOptoCh(OptoChannel::CH2);
    assertTrue(dev.isOptoChOn(OptoChannel::CH2));

    dev.turnOffOptoCh(OptoChannel::CH2);
    dev.turnOffOptoCh(OptoChannel::CH2);
    assertFalse(dev.isOptoChOn(OptoChannel::CH2));
}

test(deviceIo_unknownChannelIsRejected) {
    DeviceIO &dev = DeviceIO::getInstance();
    dev.reset();

    // Channel 1 is reserved for the IR LED and is not a valid opto channel.
    OptoChannel bad = static_cast<OptoChannel>(1);
    assertFalse(dev.turnOnOptoCh(bad));
    assertFalse(dev.turnOffOptoCh(bad));
    assertFalse(dev.isOptoChOn(bad));
}

void setup() {
    Serial.begin(115200);
    delay(kTestHostConnectDelayMs);
    aunit::TestRunner::setTimeout(30);
}

void loop() {
    aunit::TestRunner::run();
}
