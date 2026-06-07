#include "trigger_firmware/status_display.h"

#include <cstdio>
#include <cstring>

#include <Wire.h>

const char *const StatusDisplay::labels_[StatusDisplay::numLines] = {
    "Status",
    "Beh FPS",
    "Beh-mus ratio",
    "Beh exp",
    "Musc exp",
};

namespace {
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
    // Start in the pre-RUN special case: status "INITIALIZING", others blank.
    for (uint8_t i = 0; i < numLines; ++i) {
        values_[i][0] = '\0';
    }
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

void StatusDisplay::setMuscExpOff() {
    setValue(muscExpTimeLine, "OFF");
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
    // The Status line is centred in the yellow band; the remaining lines fill
    // the blue band below, numbered from 0 by their offset past statusLine.
    int16_t y;
    if (index == statusLine) {
        y = (statusBandHeight - charHeight) / 2;
    } else {
        y = statusBandHeight + blueTopMargin + (index - statusLine - 1) * rowHeight;
    }

    // Each line is "<label>: <value>", left-aligned. render() clears the panel
    // before redrawing every row, so no per-row blanking is needed. Before the
    // first RUN command the value is empty, leaving just "<label>: ".
    display_.setCursor(0, y);
    display_.print(labels_[index]);
    display_.print(": ");
    display_.print(values_[index]);
}
