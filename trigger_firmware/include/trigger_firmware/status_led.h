#pragma once

#include <cstdint>

#include "trigger_firmware/status_display.h"

/**
 * Drives the RGB status LED, which mirrors -- as a single color -- the same
 * operating status shown textually on the StatusDisplay. The LED gives an
 * at-a-glance indication that is readable from across the room, where the OLED
 * text is not.
 *
 * Color mapping (StatusDisplay::Status -> color):
 *   - initializing:          yellow (red + green)
 *   - paused:                off
 *   - streaming:             green
 *   - openRecording:         blue
 *   - scheduledRecording:    magenta (red + blue)
 *   - error:                 red
 *   - resetting:             yellow (red + green)
 *
 * Yellow flags the pre-configuration "initializing" state (no RUN command
 * received yet); "paused" (triggering suspended by the operator) is the only
 * state that leaves the LED dark.
 *
 * Hardware: a common-cathode RGB LED whose red/green/blue legs are driven by
 * the statusLed{Red,Green,Blue}Pin pins (config.h); the legs are driven with
 * PWM brightness values so the channels can be balanced independently.
 *
 * Usage: call begin() once during setup(), then setStatus() whenever the
 * operating status changes (the StatusLed shares its status source with the
 * StatusDisplay).
 */
class StatusLed {
  public:
    // PWM brightness of the three LED legs (common-cathode: 0 == off,
    // 255 == full brightness). This is the pure status-to-color decision with
    // no hardware access, so colorFor() can be unit-tested without driving real
    // pins.
    struct Color {
      uint8_t red;
      uint8_t green;
      uint8_t blue;
    };

    /** PWM calibration constants (0 == off, 255 == full). */
    inline static constexpr uint8_t kRed_ = 255;
    inline static constexpr uint8_t kGreen_ = 96;
    inline static constexpr uint8_t kBlue_ = 128;

    /** The color shown for `status` (see the class comment for the mapping). */
    static Color colorFor(StatusDisplay::Status status) {
      switch (status) {
      case StatusDisplay::Status::initializing:
        return {kRed_, kGreen_, 0}; // yellow
      case StatusDisplay::Status::paused:
        return {0, 0, 0};
      case StatusDisplay::Status::streaming:
        return {0, kGreen_, 0};
      case StatusDisplay::Status::openRecording:
        return {0, 0, kBlue_};
      case StatusDisplay::Status::scheduledRecording:
        return {kRed_, 0, kBlue_}; // magenta
      case StatusDisplay::Status::error:
        return {kRed_, 0, 0};
      case StatusDisplay::Status::resetting:
        return {kRed_, kGreen_, 0}; // yellow
      }
      return {0, 0, 0};
    }

    /** Configure the three LED pins as outputs and show the initial status. */
    void begin();

    /** Light the LED with the color mapped to `status`. */
    void setStatus(StatusDisplay::Status status);
};
