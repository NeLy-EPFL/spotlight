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
 *   - open_recording:        blue
 *   - scheduled_recording:   magenta (red + blue)
 *   - error:                 red
 *   - resetting:             yellow (red + green)
 *
 * Yellow flags the pre-configuration "initializing" state (no RUN command
 * received yet); "paused" (triggering suspended by the operator) is the only
 * state that leaves the LED dark.
 *
 * Hardware: a common-cathode RGB LED whose red/green/blue legs are driven by
 * the status_led_{red,green,blue}_pin pins (config.h); the legs are driven with
 * PWM brightness values so the channels can be balanced independently.
 *
 * Usage: call begin() once during setup(), then set_status() whenever the
 * operating status changes (the StatusLed shares its status source with the
 * StatusDisplay).
 */
class StatusLed {
  public:
    // PWM brightness of the three LED legs (common-cathode: 0 == off,
    // 255 == full brightness). This is the pure status-to-color decision with
    // no hardware access, so color_for() can be unit-tested without driving
    // real pins.
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    /** PWM calibration constants (0 == off, 255 == full). */
    inline static constexpr uint8_t red = 255;
    inline static constexpr uint8_t green = 96;
    inline static constexpr uint8_t blue = 128;

    /** The color shown for `status` (see the class comment for the mapping). */
    static Color color_for(StatusDisplay::Status status) {
        switch (status) {
        case StatusDisplay::Status::initializing:
            return {red, green, 0}; // yellow
        case StatusDisplay::Status::paused:
            return {0, 0, 0};
        case StatusDisplay::Status::streaming:
            return {0, green, 0};
        case StatusDisplay::Status::open_recording:
            return {0, 0, blue};
        case StatusDisplay::Status::scheduled_recording:
            return {red, 0, blue}; // magenta
        case StatusDisplay::Status::error:
            return {red, 0, 0};
        case StatusDisplay::Status::resetting:
            return {red, green, 0}; // yellow
        }
        return {0, 0, 0};
    }

    /** Configure the three LED pins as outputs and show the initial status. */
    void begin();

    /** Light the LED with the color mapped to `status`. */
    void set_status(StatusDisplay::Status status);
};
