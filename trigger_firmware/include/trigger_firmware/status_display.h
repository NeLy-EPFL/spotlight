#pragma once

#include <cstddef>

#include <Adafruit_SSD1306.h>

/**
 * This class defines the interface for controlling the display used to show
 * current triggering configurations.
 *
 * Hardware: Midas MDOB128064WV-YBI OLED display (128x64 pixels) with built-in
 * I2C, controlled by Arduino Nano ESP32 via SDA (A4) and SCL (A5) pins.
 *
 * Format of displayed text (in a monospaced font):
 *
 *     Status: x
 *     Beh FPS: x
 *     Beh-mus ratio: x:1
 *     Beh exp: x us
 *     Mus exp: x us
 *
 * For each row, the x is:
 *   - For "Status":
 *     - "PAUSED" when the pause override is on (via physical on/off switch)
 *     - "STREAMING" after every command where recording/is_recording is false
 *     - "OPEN RECORDING" after every command where recording/is_recording is
 *       true and recording/op_sequence is empty
 *     - "SCHEDULED RECORDING" after every command where recording/is_recording
 *       is true and recording/op_sequence is nonempty
 *     - "ERROR" if the controller is in an error state
 *     - "RESETTING" briefly, after a RESET command, just before the controller
 *       reboots (all other lines blank)
 *   - For "Beh FPS": the beh_frame_rate parameter of the most recent RUN
 *     command
 *   - For "Beh-mus ratio": the beh_musc_sync_ratio parameter of the most recent
 *     RUN command, followed by ":1"; or "N/A" when that command had
 *     enable_muscle false (muscle imaging disabled, so the ratio is
 * meaningless)
 *   - For "Beh exp": the beh_exp_time parameter of the most recent RUN
 *     command, followed by " us"
 *   - For "Mus exp": the musc_eff_exp_time parameter of the most recent
 *     RUN command, followed by " us"; or "OFF" when that command had
 *     enable_muscle false (muscle imaging disabled)
 * The Status line should occupy the top 16 pixels (which are in yellow), and
 * the rest should occupy the bottom 48 pixels (which are in blue).
 *
 * Special case: Before the first RUN command is received, "Status" should show
 * "INITIALIZING" and the other fields should be empty (no units or "x").
 *
 * Usage: call begin() once during setup(), then use the per-line setters to
 * update individual values and call render() to push the cached state to the
 * panel. The setters only touch an in-memory cache, so several of them can be
 * batched (e.g. on a RUN command) before a single render().
 */
class StatusDisplay {
  public:
    /** Value shown on the "Status" line. */
    enum class Status {
        initializing,
        paused,
        streaming,
        open_recording,
        scheduled_recording,
        error,
        resetting,
    };

    StatusDisplay();

    /**
     * Initialize the OLED over I2C and draw the initial screen (status
     * "INITIALIZING", all other values blank). Returns false if the panel did
     * not acknowledge on the I2C bus.
     */
    bool begin();

    // Per-line value setters. Each only updates the cached text for its line;
    // call render() afterwards to push the changes to the panel.
    void set_status(Status status);
    void set_beh_frame_rate(unsigned long fps);
    void set_beh_musc_sync_ratio(unsigned long ratio);
    /** Show "N/A" on the "Beh-mus ratio" line (muscle imaging disabled). */
    void set_beh_musc_ratio_na();
    void set_beh_exp_time(unsigned long us);
    void set_musc_exp_time(unsigned long us);
    /** Show "OFF" on the "Mus exp" line (muscle imaging disabled). */
    void set_musc_exp_off();

    /**
     * Blank every line (the status line and all parameter lines). Only touches
     * the in-memory cache; call render() to push the cleared state to the
     * panel. Used on a software reset, where the status is then set to
     * "RESETTING".
     */
    void clear();

    /** Redraw the whole screen from the cached line values. */
    void render();

  private:
    // Lines in top-to-bottom display order. num_lines doubles as the line
    // count.
    enum Line {
        status_line,
        beh_frame_rate_line,
        beh_musc_ratio_line,
        beh_exp_time_line,
        musc_exp_time_line,
        num_lines,
    };

    static constexpr uint8_t screen_width = 128;
    static constexpr uint8_t screen_height = 64;
    static constexpr int8_t reset_pin = -1;      // no dedicated reset pin
    static constexpr uint8_t i2c_address = 0x3C; // Midas MDOB128064WV-YBI
    static constexpr uint8_t char_height = 8;    // default GFX font at size 1
    // The panel is physically two-tone: the top 16 px are yellow and the
    // bottom 48 px are blue. The Status line sits alone in the yellow band
    // (vertically centred); the four RUN-parameter lines share the blue band,
    // spaced at a 12 px pitch starting 2 px below the seam (16 + 2 + 3 * 12 =
    // 54, leaving the 8 px font room within the 64 px panel).
    static constexpr uint8_t status_band_height = 16;
    static constexpr uint8_t row_height = 12;
    static constexpr uint8_t blue_top_margin = 2;
    static constexpr size_t value_buffer_size = 24;

    // Fixed labels, indexed by Line.
    static const char *const labels_[num_lines];

    Adafruit_SSD1306 display_;
    char values_[num_lines][value_buffer_size];

    // Copy text into the cache for one line (truncating if necessary).
    void set_value(Line line, const char *text);
    // Draw one label/value row at its fixed vertical position.
    void draw_row(uint8_t index);
};
