// Hardware test: ArduinoCommunication (trigger controller) init-reset-destroy
// cycle.  The reset command reboots the firmware (esp_restart()); the
// communication thread reconnects automatically once the USB CDC link comes
// back, which takes ~3-5 seconds.
// Requires the Arduino Nano ESP32 trigger controller to be powered and
// connected.
// Run with SPOTLIGHT_PROFILE_DIR set to a valid profile directory.

#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "recorder/peripherals/arduino_communication.h"
#include "test_hardware_helpers.h"

namespace {
// Time to wait after reset() for the firmware to reboot and reconnect.
constexpr int reboot_wait_seconds = 6;
} // namespace

TEST(ArduinoCommunicationHardwareTest, InitResetDestroy) {
    REQUIRE_SPOTLIGHT_PROFILE_DIR(profile_dir);
    RecorderConfig config = load_recorder_config(profile_dir);

    const std::string port_name = find_arduino_port_name(config);
    ASSERT_FALSE(port_name.empty()) << "Arduino not found on any serial port";

    // Constructor starts the background communication thread.
    ArduinoCommunication arduino(port_name);

    // reset() sends a RESET command; the firmware calls esp_restart() and
    // re-enumerates on USB.  The communication thread handles reconnection.
    arduino.reset();

    // Wait for the firmware to reboot and the serial link to come back.
    std::this_thread::sleep_for(std::chrono::seconds(reboot_wait_seconds));

    // Clean shutdown: drains the outgoing queue and joins the comm thread.
    arduino.stop_communication();

    // Destructor runs here.
}
