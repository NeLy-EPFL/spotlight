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
 *     v0.1.0 b260601-1430
 *     Status         x
 *     Beh FPS        x
 *     B-M ratio      x:1
 *     Beh exp time   x us
 *     Musc exp time  x us
 *
 * The top row is a full-width version/build line with no label (see below).
 * For the remaining rows, the x is:
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
 *   - For "B-M ratio": the behMuscSyncRatio parameter of the most recent RUN
 *     command, followed by ":1"
 *   - For "Beh exp time": the behExpTime parameter of the most recent RUN
 *     command, followed by " us"
 *   - For "Musc exp time": the muscEffExpTime parameter of the most recent
 *     RUN command, followed by " us"
 *
 * Version line: "v<version> b<YYMMDD>-<HHmm>", where <version> is
 * config::firmwareVersion and the build code is the compiler host's LOCAL
 * date/time (from __DATE__/__TIME__), e.g. "v0.1.0 b260601-1430". This line is
 * fixed at startup and never changes at runtime.
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
        versionLine,
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
    static constexpr uint8_t charWidth = 6;     // default GFX font at size 1
    static constexpr uint8_t charHeight = 8;    // default GFX font at size 1
    // Six rows of the 6x8 default font fit in the 64 px panel with room to
    // spare, so the row pitch is relaxed to 10 px (2 px top margin + 6 * 10 =
    // 62, leaving the 8 px font a 2 px gap between rows).
    static constexpr uint8_t rowHeight = 10;
    static constexpr uint8_t topMargin = 2;
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
