#pragma once

#include <Arduino.h> // for the A0 pin constant

namespace config {
// Serial IO
inline constexpr int serialBaudRate = 115200;
inline constexpr int incomingCmdBufferSize = 16384;

// Default streaming parameters.
// The controller applies these at startup so it streams immediately, before the
// first STREAM command arrives from the host.
inline constexpr bool defaultEnableMuscle = false;
inline constexpr unsigned int defaultBehFrameRate = 25;   // fps
inline constexpr unsigned int defaultBehExpTime = 1000;   // us

// Pin assignments
inline constexpr int onOffSwitchPin = D8;

inline constexpr int behCamPin = D10;
inline constexpr int irLEDPin = D5;

inline constexpr int muscCamTriggerPin = D12;
// Use A0 instead of D13 for muscle camera status because D13 is shared with
// LED_BUILTIN/SCK. A0 is a digital-capable pin.
inline constexpr int muscCamStatusPin = A0;
inline constexpr int blueLEDPin = D11;

inline constexpr int optoCh2Pin = D6;
inline constexpr int optoCh3Pin = D7;

inline constexpr int statusLedRedPin = D2;
inline constexpr int statusLedGreenPin = D3;
inline constexpr int statusLedBluePin = D4;
} // namespace config
