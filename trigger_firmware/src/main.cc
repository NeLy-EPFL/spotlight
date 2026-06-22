// Excluded from on-device unit-test builds (pio test): PlatformIO defines
// PIO_UNIT_TESTING then, and each test program under test/ provides its own
// setup()/loop(). Compiling this file too would give two definitions of each.
#ifndef PIO_UNIT_TESTING

#include <Arduino.h>

#include "trigger_firmware/trigger_controller.h"

// The arduino-esp32 core (cores/esp32/main.cpp) calls setup() once and then
// loop() forever. The whole firmware is the single TriggerController object.
//
// It is held as a function-local static so it is constructed on the first
// setup() call -- i.e. after the Arduino core has initialized -- rather than
// during C++ global construction, which runs before the GPIO/serial peripherals
// are ready.

namespace {
TriggerController &controller() {
    static TriggerController instance;
    return instance;
}
} // namespace

void setup() {
    controller().begin();
}

void loop() {
    controller().update();
}

#endif // PIO_UNIT_TESTING
