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
inline constexpr int musc_cam_acquire_enable_pin = A1;
inline constexpr int calcium_cam_status_pin = A2;
inline constexpr int fiducial_cam_status_pin = A3;

// Time the acquire-enable line is held LOW before a measurement, so that both
// cameras are idle when it is released.
inline constexpr unsigned long acquire_disable_time_ms = 1000;

// Upper bound on the number of frames per measurement (4 edge buffers of this
// many 4-byte entries must fit in the heap).
inline constexpr size_t max_n_frames = 10000;

// A measurement is aborted if a camera produces no status edge for this many
// shutter-open times (one frame period is about one shutter-open time).
inline constexpr unsigned long edge_timeout_factor = 2;

// Binary response protocol, mirrored in the C++ program (cpp/src/main.cc).
// Every response starts with a header of three little-endian uint32 values:
// magic, status, and number of frames. If the status is ok, four arrays of
// n_frames uint32 values follow: calcium on, calcium off, fiducial on, and
// fiducial off edge times, in us since acquire enable went HIGH.
inline constexpr uint32_t response_magic = 0x54465244; // "DRFT" on the wire

// OLED display (Midas MDOB128064WV-YBI), same as trigger_firmware.
inline constexpr int screen_width = 128;
inline constexpr int screen_height = 64;
inline constexpr uint8_t display_i2c_address = 0x3C;
} // namespace config
