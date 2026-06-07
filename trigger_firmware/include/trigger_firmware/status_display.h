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
 *     Musc exp: x us
 *
 * For each row, the x is:
 *   - For "Status":
 *     - "PAUSED" when the pause override is on (via physical on/off switch)
 *     - "STREAMING" after every command where recording/isRecording is false
 *     - "OPEN RECORDING" after every command where recording/isRecording is
 *       true and recording/opSequence is empty
 *     - "SCHEDULED RECORDING" after every command where recording/isRecording
 *       is true and recording/opSequence is nonempty
 *     - "ERROR" if the controller is in an error state
 *   - For "Beh FPS": the behFrameRate parameter of the most recent RUN
 *     command
 *   - For "Beh-mus ratio": the behMuscSyncRatio parameter of the most recent RUN
 *     command, followed by ":1"
 *   - For "Beh exp": the behExpTime parameter of the most recent RUN
 *     command, followed by " us"
 *   - For "Musc exp": the muscEffExpTime parameter of the most recent
 *     RUN command, followed by " us"
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
        openRecording,
        scheduledRecording,
        error,
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
    void setStatus(Status status);
    void setBehFrameRate(unsigned long fps);
    void setBehMuscSyncRatio(unsigned long ratio);
    void setBehExpTime(unsigned long us);
    void setMuscExpTime(unsigned long us);

    /** Redraw the whole screen from the cached line values. */
    void render();

  private:
    // Lines in top-to-bottom display order. numLines doubles as the line count.
    enum Line {
        statusLine,
        behFrameRateLine,
        behMuscRatioLine,
        behExpTimeLine,
        muscExpTimeLine,
        numLines,
    };

    static constexpr uint8_t screenWidth = 128;
    static constexpr uint8_t screenHeight = 64;
    static constexpr int8_t resetPin = -1;      // no dedicated reset pin
    static constexpr uint8_t i2cAddress = 0x3C; // Midas MDOB128064WV-YBI
    static constexpr uint8_t charHeight = 8; // default GFX font at size 1
    // The panel is physically two-tone: the top 16 px are yellow and the
    // bottom 48 px are blue. The Status line sits alone in the yellow band
    // (vertically centred); the four RUN-parameter lines share the blue band,
    // spaced at a 12 px pitch starting 2 px below the seam (16 + 2 + 3 * 12 =
    // 54, leaving the 8 px font room within the 64 px panel).
    static constexpr uint8_t statusBandHeight = 16;
    static constexpr uint8_t rowHeight = 12;
    static constexpr uint8_t blueTopMargin = 2;
    static constexpr size_t valueBufferSize = 24;

    // Fixed labels, indexed by Line.
    static const char *const labels_[numLines];

    Adafruit_SSD1306 display_;
    char values_[numLines][valueBufferSize];

    // Copy text into the cache for one line (truncating if necessary).
    void setValue(Line line, const char *text);
    // Draw one label/value row at its fixed vertical position.
    void drawRow(uint8_t index);
};
