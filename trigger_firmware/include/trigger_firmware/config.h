#pragma once

namespace config {
// Firmware version, shown (with the build timestamp) on the top row of the
// status display as "v<version> b<YYMMDD>-<HHmm>". Bump this on each
// meaningful firmware change so the build running on the device can be
// identified at a glance.
inline constexpr char firmwareVersion[] = "0.4.0";

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
