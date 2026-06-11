// On-device AUnit tests for DeviceIO.
//
// DeviceIO owns the physical outputs, so these run on the actual Arduino Nano
// ESP32 (uploaded and executed by `pio test`, see the project README). They
// exercise the cached output-state bookkeeping that the rest of the firmware
// relies on: the idempotent setters, the OptoChannel::all aggregation, and
// reset(). They do not assert on electrical pin levels, only on the logical
// state DeviceIO reports back through is_opto_ch_on().

#include <AUnit.h>
#include <Arduino.h>

#include "trigger_firmware/device_io.h"

// The Arduino Nano ESP32's native USB CDC reports "connected" immediately and
// is not reset when the port is opened, so the board would otherwise print its
// results before the test host (`pio test`) attaches and they would be lost.
// Delaying the start of the run leaves time for the host to open the port
// first. See the firmware test instructions in the project README.
static constexpr unsigned long test_host_connect_delay_ms = 20000;

// There is a single physical output set, so DeviceIO is a singleton. Each test
// starts from reset() to get a known state regardless of run order.

test(deviceIo_resetTurnsEverythingOff) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.turn_on_opto_ch(OptoChannel::all);
    dev.reset();
    assertFalse(dev.is_opto_ch_on(OptoChannel::ch2));
    assertFalse(dev.is_opto_ch_on(OptoChannel::ch3));
    assertFalse(dev.is_opto_ch_on(OptoChannel::all));
}

test(deviceIo_turnOnSingleChannel) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.reset();

    assertTrue(dev.turn_on_opto_ch(OptoChannel::ch2));
    assertTrue(dev.is_opto_ch_on(OptoChannel::ch2));
    assertFalse(dev.is_opto_ch_on(OptoChannel::ch3));
    // all is on only when both channels are on.
    assertFalse(dev.is_opto_ch_on(OptoChannel::all));
}

test(deviceIo_turnOnAllThenQueryParts) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.reset();

    assertTrue(dev.turn_on_opto_ch(OptoChannel::all));
    assertTrue(dev.is_opto_ch_on(OptoChannel::ch2));
    assertTrue(dev.is_opto_ch_on(OptoChannel::ch3));
    assertTrue(dev.is_opto_ch_on(OptoChannel::all));
}

test(deviceIo_turnOffSingleChannel) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.reset();
    dev.turn_on_opto_ch(OptoChannel::all);

    assertTrue(dev.turn_off_opto_ch(OptoChannel::ch3));
    assertTrue(dev.is_opto_ch_on(OptoChannel::ch2));
    assertFalse(dev.is_opto_ch_on(OptoChannel::ch3));
    assertFalse(dev.is_opto_ch_on(OptoChannel::all));
}

test(deviceIo_settersAreIdempotent) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.reset();

    // Calling a setter repeatedly must leave the reported state unchanged.
    dev.turn_on_opto_ch(OptoChannel::ch2);
    dev.turn_on_opto_ch(OptoChannel::ch2);
    assertTrue(dev.is_opto_ch_on(OptoChannel::ch2));

    dev.turn_off_opto_ch(OptoChannel::ch2);
    dev.turn_off_opto_ch(OptoChannel::ch2);
    assertFalse(dev.is_opto_ch_on(OptoChannel::ch2));
}

test(deviceIo_unknownChannelIsRejected) {
    DeviceIO &dev = DeviceIO::get_instance();
    dev.reset();

    // Channel 1 is reserved for the IR LED and is not a valid opto channel.
    OptoChannel bad = static_cast<OptoChannel>(1);
    assertFalse(dev.turn_on_opto_ch(bad));
    assertFalse(dev.turn_off_opto_ch(bad));
    assertFalse(dev.is_opto_ch_on(bad));
}

void setup() {
    Serial.begin(115200);
    delay(test_host_connect_delay_ms);
    aunit::TestRunner::setTimeout(30);
}

void loop() {
    aunit::TestRunner::run();
}
