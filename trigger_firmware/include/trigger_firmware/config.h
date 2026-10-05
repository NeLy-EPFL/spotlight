#pragma once

#include <Arduino.h> // for the D and A pin constants

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

// Pin assignments. D10, D11, D12, and D13 (LED_BUILTIN/SCK) are unused; A4/A5
// are the OLED's I2C bus (see status_display.h). All A pins are used as digital
// GPIOs.
inline constexpr int on_off_switch_pin = D8;

inline constexpr int beh_cam_pin = A6;
inline constexpr int ir_led_pin = D5;

// The muscle camera trigger and acquire-enable lines each drive both PCO
// cameras (calcium and fiducial) in parallel.
inline constexpr int musc_cam_trigger_pin = A0;
inline constexpr int musc_cam_acquire_enable_pin = A1;
inline constexpr int calcium_cam_status_pin = A2;
inline constexpr int fiducial_cam_status_pin = A3;
inline constexpr int blue_led_pin = A7;

inline constexpr int opto_ch2_pin = D6;
inline constexpr int opto_ch3_pin = D7;

inline constexpr int status_led_red_pin = D2;
inline constexpr int status_led_green_pin = D3;
inline constexpr int status_led_blue_pin = D4;
} // namespace config
