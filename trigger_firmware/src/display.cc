#include "trigger_firmware/display.h"

#include <cstdio>
#include <cstring>

#include <Wire.h>

#include "trigger_firmware/config.h"

const char *const StatusDisplay::labels_[StatusDisplay::numLines] = {
    "", // version line: full-width, no label
    "Status",
    "PCO cam mode",
    "Beh FPS",
    "B-M ratio",
    "Beh exp time",
    "Musc exp time",
};

namespace {
// Convert a three-letter __DATE__ month abbreviation ("Jan".."Dec") to its
// 1-based month number, or 0 if unrecognized.
int monthNumber(const char *abbrev) {
    static const char *const months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (int i = 0; i < 12; ++i) {
        if (std::strncmp(abbrev, months + i * 3, 3) == 0) {
            return i + 1;
        }
    }
    return 0;
}

// Map a Status enum value to the text shown on the "Status" line.
const char *statusText(StatusDisplay::Status status) {
    switch (status) {
    case StatusDisplay::Status::initializing:
        return "INITIALIZING";
    case StatusDisplay::Status::paused:
        return "PAUSED";
    case StatusDisplay::Status::streaming:
        return "STREAMING";
    case StatusDisplay::Status::openRecording:
        return "OPEN RECORDING";
    case StatusDisplay::Status::scheduledRecording:
        return "SCHEDULED RECORDING";
    case StatusDisplay::Status::error:
        return "ERROR";
    }
    return "";
}
} // namespace

StatusDisplay::StatusDisplay()
    : display_(screenWidth, screenHeight, &Wire, resetPin) {
    // Start in the pre-SET special case: status "INITIALIZING", others blank.
    for (uint8_t i = 0; i < numLines; ++i) {
        values_[i][0] = '\0';
    }

    // The version line is fixed for the lifetime of the firmware: the manual
    // version plus a compact build timestamp in the compiler host's LOCAL
    // time, e.g. "v0.1.0 b260601-1430". __DATE__ is "Mmm DD YYYY" (the day is
    // space-padded, so map a leading space to '0') and __TIME__ is "HH:MM:SS".
    const char *const date = __DATE__;
    const char *const time = __TIME__;
    const char dayTens = date[4] == ' ' ? '0' : date[4];
    std::snprintf(values_[versionLine], valueBufferSize,
                  "v%s b%c%c%02d%c%c-%c%c%c%c", config::firmwareVersion,
                  date[9], date[10], monthNumber(date), dayTens, date[5],
                  time[0], time[1], time[3], time[4]);

    setStatus(Status::initializing);
}

bool StatusDisplay::begin() {
    if (!display_.begin(SSD1306_SWITCHCAPVCC, i2cAddress)) {
        return false;
    }
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    display_.setTextWrap(false);
    render();
    return true;
}

void StatusDisplay::setStatus(Status status) {
    setValue(statusLine, statusText(status));
}

void StatusDisplay::setPcoCamContinuous(bool continuous) {
    setValue(pcoCamModeLine, continuous ? "CONT" : "TRIG");
}

void StatusDisplay::setBehFrameRate(unsigned long fps) {
    std::snprintf(values_[behFrameRateLine], valueBufferSize, "%lu", fps);
}

void StatusDisplay::setBehMuscSyncRatio(unsigned long ratio) {
    std::snprintf(values_[behMuscRatioLine], valueBufferSize, "%lu:1", ratio);
}

void StatusDisplay::setBehExpTime(unsigned long us) {
    std::snprintf(values_[behExpTimeLine], valueBufferSize, "%lu us", us);
}

void StatusDisplay::setMuscExpTime(unsigned long us) {
    std::snprintf(values_[muscExpTimeLine], valueBufferSize, "%lu us", us);
}

void StatusDisplay::render() {
    display_.clearDisplay();
    for (uint8_t i = 0; i < numLines; ++i) {
        drawRow(i);
    }
    display_.display();
}

void StatusDisplay::setValue(Line line, const char *text) {
    std::strncpy(values_[line], text, valueBufferSize - 1);
    values_[line][valueBufferSize - 1] = '\0';
}

void StatusDisplay::drawRow(uint8_t index) {
    const int16_t y = topMargin + index * rowHeight;

    // A row with no label (the version header) shows its value as a full-width,
    // left-aligned line instead of the label/value column layout.
    if (labels_[index][0] == '\0') {
        display_.setCursor(0, y);
        display_.print(values_[index]);
        return;
    }

    // Label, left-aligned.
    display_.setCursor(0, y);
    display_.print(labels_[index]);

    // Value, right-aligned. Blank the value column first so a long label cannot
    // bleed underneath it (the value always takes precedence).
    const char *value = values_[index];
    const size_t len = std::strlen(value);
    if (len == 0) {
        return;
    }
    const int16_t valueX =
        screenWidth - static_cast<int16_t>(len) * charWidth;
    display_.fillRect(valueX, y, screenWidth - valueX, charHeight,
                      SSD1306_BLACK);
    display_.setCursor(valueX, y);
    display_.print(value);
}
