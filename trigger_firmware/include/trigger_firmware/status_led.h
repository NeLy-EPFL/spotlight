#pragma once

#include "trigger_firmware/status_display.h"

/**
 * Drives the RGB status LED, which mirrors -- as a single color -- the same
 * operating status shown textually on the StatusDisplay. The LED gives an
 * at-a-glance indication that is readable from across the room, where the OLED
 * text is not.
 *
 * Color mapping (StatusDisplay::Status -> color):
 *   - initializing / paused: off
 *   - streaming:             green
 *   - openRecording:         blue
 *   - scheduledRecording:    magenta (red + blue)
 *   - error:                 red
 *
 * The "initializing" pre-configuration state is indicated the same way as
 * "paused" (LED off): in both cases no triggering is happening.
 *
 * Hardware: a common-cathode RGB LED whose red/green/blue legs are driven by
 * the statusLed{Red,Green,Blue}Pin pins (config.h); driving a leg HIGH lights
 * it.
 *
 * Usage: call begin() once during setup(), then setStatus() whenever the
 * operating status changes (the StatusLed shares its status source with the
 * StatusDisplay).
 */
class StatusLed {
  public:
    /** Configure the three LED pins as outputs and turn the LED off. */
    void begin();

    /** Light the LED with the color mapped to `status`. */
    void setStatus(StatusDisplay::Status status);
};
