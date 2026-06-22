#include "trigger_firmware/status_display.h"

#include <cstdio>
#include <cstring>

#include <Wire.h>

const char *const StatusDisplay::labels_[StatusDisplay::num_lines] = {
    "Status",
    "Beh FPS",
    "Beh-mus ratio",
    "Beh exp",
    "Mus exp",
};

namespace {
// Map a Status enum value to the text shown on the "Status" line.
const char *status_text(StatusDisplay::Status status) {
    switch (status) {
    case StatusDisplay::Status::initializing:
        return "INITIALIZING";
    case StatusDisplay::Status::paused:
        return "PAUSED";
    case StatusDisplay::Status::streaming:
        return "STREAMING";
    case StatusDisplay::Status::open_recording:
        return "OPEN RECORDING";
    case StatusDisplay::Status::scheduled_recording:
        return "SCHEDULED RECORDING";
    case StatusDisplay::Status::error:
        return "ERROR";
    case StatusDisplay::Status::resetting:
        return "RESETTING";
    }
    return "";
}
} // namespace

StatusDisplay::StatusDisplay()
    : display_(screen_width, screen_height, &Wire, reset_pin) {
    // Start in the pre-RUN special case: status "INITIALIZING", others blank.
    clear();
    set_status(Status::initializing);
}

bool StatusDisplay::begin() {
    if (!display_.begin(SSD1306_SWITCHCAPVCC, i2c_address)) {
        return false;
    }
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    display_.setTextWrap(false);
    render();
    return true;
}

void StatusDisplay::set_status(Status status) {
    set_value(status_line, status_text(status));
}

void StatusDisplay::set_beh_frame_rate(unsigned long fps) {
    std::snprintf(values_[beh_frame_rate_line], value_buffer_size, "%lu", fps);
}

void StatusDisplay::set_beh_musc_sync_ratio(unsigned long ratio) {
    std::snprintf(
        values_[beh_musc_ratio_line], value_buffer_size, "%lu:1", ratio);
}

void StatusDisplay::set_beh_musc_ratio_na() {
    set_value(beh_musc_ratio_line, "N/A");
}

void StatusDisplay::set_beh_exp_time(unsigned long us) {
    std::snprintf(values_[beh_exp_time_line], value_buffer_size, "%lu us", us);
}

void StatusDisplay::set_musc_exp_time(unsigned long us) {
    std::snprintf(values_[musc_exp_time_line], value_buffer_size, "%lu us", us);
}

void StatusDisplay::set_musc_exp_off() {
    set_value(musc_exp_time_line, "OFF");
}

void StatusDisplay::clear() {
    for (uint8_t i = 0; i < num_lines; ++i) {
        values_[i][0] = '\0';
    }
}

void StatusDisplay::render() {
    display_.clearDisplay();
    for (uint8_t i = 0; i < num_lines; ++i) {
        draw_row(i);
    }
    display_.display();
}

void StatusDisplay::render_off() {
    display_.clearDisplay();
    display_.display();
}

void StatusDisplay::set_value(Line line, const char *text) {
    std::strncpy(values_[line], text, value_buffer_size - 1);
    values_[line][value_buffer_size - 1] = '\0';
}

void StatusDisplay::draw_row(uint8_t index) {
    // The Status line is centred in the yellow band; the remaining lines fill
    // the blue band below, numbered from 0 by their offset past status_line.
    int16_t y;
    if (index == status_line) {
        y = (status_band_height - char_height) / 2;
    } else {
        y = status_band_height + blue_top_margin +
            (index - status_line - 1) * row_height;
    }

    // Each line is "<label>: <value>", left-aligned. render() clears the panel
    // before redrawing every row, so no per-row blanking is needed. Before the
    // first RUN command the value is empty, leaving just "<label>: ".
    display_.setCursor(0, y);
    display_.print(labels_[index]);
    display_.print(": ");
    display_.print(values_[index]);
}
