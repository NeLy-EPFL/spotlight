#pragma once

#include <Arduino.h> // for the A pin constants

#include <cstddef>
#include <cstdint>

namespace config {
// Serial IO (the baud rate is ignored by the native USB CDC port)
inline constexpr int serial_baud_rate = 115200;
inline constexpr size_t command_buffer_size = 64;

// Pin assignments, same as trigger_firmware/include/trigger_firmware/config.h.
// The acquire-enable line drives both PCO cameras in parallel.
inline constexpr int musc_cam_acquire_enable_pin = A2;
inline constexpr int calcium_cam_status_pin = A1;
inline constexpr int fiducial_cam_status_pin = A0;

// Time the acquire-enable line is held LOW before a measurement, so that both
// cameras are idle when it is released.
inline constexpr unsigned long acquire_disable_time_ms = 1000;

// Binary response protocol, mirrored in the C++ program (cpp/src/main.cc).
// Every response starts with a header of three little-endian uint32 values:
// magic, status, and number of samples. If the status is ok, the samples
// follow, packed 4 per byte (the last byte is zero-padded): sample i is in bits
// 2 * (i % 4) (calcium) and 2 * (i % 4) + 1 (fiducial) of byte i / 4.
inline constexpr uint32_t response_magic = 0x434e5352; // "RSNC" on the wire
inline constexpr uint32_t samples_per_byte = 4;

// OLED display (Midas MDOB128064WV-YBI), same as trigger_firmware.
inline constexpr int screen_width = 128;
inline constexpr int screen_height = 64;
inline constexpr uint8_t display_i2c_address = 0x3C;
} // namespace config
