#pragma once

#include <Arduino.h> // for the A0 pin constant

namespace config {
// Serial IO
inline constexpr int serialBaudRate = 115200;
inline constexpr int incomingCmdBufferSize = 16384;

// Default streaming parameters. The controller applies these at startup so it
// streams immediately, before the first STREAM command arrives from the host.
inline constexpr bool defaultEnableMuscle = false;
inline constexpr unsigned int defaultBehFrameRate = 25;   // fps
inline constexpr unsigned int defaultBehExpTime = 1000;   // us

// Pin assignments
inline constexpr int onOffSwitchPin = 8;

inline constexpr int behCamPin = 10;
inline constexpr int irLEDPin = 5;

inline constexpr int muscCamTriggerPin = 12;
// A0: a plain digital-capable GPIO with an internal pull-down, chosen instead
// of D13 because D13 is shared with LED_BUILTIN/SCK. See device_io.cc for
// why the status line is read with INPUT_PULLDOWN.
inline constexpr int muscCamStatusPin = A0;
inline constexpr int blueLEDPin = 11;

inline constexpr int optoCh2Pin = 6;
inline constexpr int optoCh3Pin = 7;

inline constexpr int statusLedRedPin = 2;
inline constexpr int statusLedGreenPin = 3;
inline constexpr int statusLedBluePin = 4;
} // namespace config
