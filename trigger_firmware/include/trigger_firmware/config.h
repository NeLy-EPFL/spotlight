#pragma once

namespace config {
// Serial IO
inline constexpr int serialBaudRate = 115200;
inline constexpr int incomingCmdBufferSize = 16384;

// Pin assignments
inline constexpr int onOffSwitchPin = 8;

inline constexpr int behCamPin = 10;
inline constexpr int irLEDPin = 5;

inline constexpr int muscCamTriggerPin = 12;
inline constexpr int muscCamStatusPin = 13;
inline constexpr int blueLEDPin = 11;

inline constexpr int optoCh2Pin = 6;
inline constexpr int optoCh3Pin = 7;

inline constexpr int statusLedRedPin = 2;
inline constexpr int statusLedGreenPin = 3;
inline constexpr int statusLedBluePin = 4;
} // namespace config
