#pragma once

#include <Arduino.h> // for the A0 pin constant

namespace config {
// Serial IO
inline constexpr int serial_baud_rate = 115200;
inline constexpr int incoming_cmd_buffer_size = 16384;

// Default streaming parameters.
// The controller applies these at startup so it streams immediately, before the
// first STREAM command arrives from the host.
inline constexpr bool default_enable_muscle = false;
inline constexpr unsigned int default_beh_frame_rate = 25; // fps
inline constexpr unsigned int default_beh_exp_time = 1000; // us

// Pin assignments
inline constexpr int on_off_switch_pin = D8;

inline constexpr int beh_cam_pin = D10;
inline constexpr int ir_led_pin = D5;

inline constexpr int musc_cam_trigger_pin = D12;
// Use A0 instead of D13 for muscle camera status because D13 is shared with
// LED_BUILTIN/SCK. A0 is a digital-capable pin.
inline constexpr int musc_cam_status_pin = A0;
inline constexpr int blue_led_pin = D11;

inline constexpr int opto_ch2_pin = D6;
inline constexpr int opto_ch3_pin = D7;

inline constexpr int status_led_red_pin = D2;
inline constexpr int status_led_green_pin = D3;
inline constexpr int status_led_blue_pin = D4;
} // namespace config
