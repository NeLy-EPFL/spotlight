// Samples the status (SMA #4, common time) lines of both PCO muscle cameras at
// a fixed rate after releasing their shared acquire-enable line, and sends the
// whole sampled sequence to the host. Like trigger_firmware, it uses a plain
// loop with micros() and digitalRead(). See ../README.md.
//
// Serial protocol: the host sends "<duration_us> <sample_period_us>\n" (e.g.
// "15166666 1\n"), and the firmware answers with a binary response (see
// config::response_magic).

#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include <cstdio>

#include "pco_clock_drift/config.h"

namespace {
enum class Status : uint32_t {
    ok = 0,
    bad_command = 1,
    buffer_too_small = 2,
};

// Packed samples (see config::response_magic), allocated in PSRAM in setup().
uint8_t *sample_bytes = nullptr;
size_t max_n_sample_bytes = 0;

Adafruit_SSD1306 display(
    config::screen_width, config::screen_height, &Wire, /*rst_pin=*/-1);

char command[config::command_buffer_size];
size_t command_length = 0;

void show_title() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, config::display_i2c_address)) {
        return;
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("PCO CAM");
    display.println("CLOCK");
    display.println("DRIFT TEST");
    display.display();
}

// Releases acquire enable and samples both status lines n_samples times, every
// sample_period_us.
void sample(uint32_t n_samples, unsigned long sample_period_us) {
    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    delay(config::acquire_disable_time_ms);
    digitalWrite(config::musc_cam_acquire_enable_pin, HIGH);

    unsigned long next_sample_us = micros();
    uint8_t byte = 0;
    for (uint32_t i = 0; i < n_samples; ++i) {
        while (static_cast<long>(micros() - next_sample_us) < 0) {
        }
        next_sample_us += sample_period_us;
        const uint8_t levels =
            digitalRead(config::calcium_cam_status_pin) |
            (digitalRead(config::fiducial_cam_status_pin) << 1);
        const uint32_t slot = i % config::samples_per_byte;
        byte |= levels << (2 * slot);
        if (slot == config::samples_per_byte - 1 || i == n_samples - 1) {
            sample_bytes[i / config::samples_per_byte] = byte;
            byte = 0;
        }
    }

    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
}

void send_response(Status status, uint32_t n_samples, size_t n_bytes) {
    const uint32_t header[] = {
        config::response_magic,
        static_cast<uint32_t>(status),
        n_samples,
    };
    Serial.write(reinterpret_cast<const uint8_t *>(header), sizeof(header));
    Serial.write(sample_bytes, n_bytes);
    Serial.flush();
}

void handle_command() {
    unsigned long duration_us = 0;
    unsigned long sample_period_us = 0;
    const bool is_valid =
        std::sscanf(command, "%lu %lu", &duration_us, &sample_period_us) ==
            2 &&
        duration_us > 0 && sample_period_us > 0;
    if (!is_valid) {
        send_response(Status::bad_command, 0, 0);
        return;
    }
    const uint32_t n_samples = duration_us / sample_period_us;
    const size_t n_bytes =
        (n_samples + config::samples_per_byte - 1) / config::samples_per_byte;
    if (n_bytes > max_n_sample_bytes) {
        send_response(Status::buffer_too_small, 0, 0);
        return;
    }
    sample(n_samples, sample_period_us);
    send_response(Status::ok, n_samples, n_bytes);
}
} // namespace

void setup() {
    pinMode(config::musc_cam_acquire_enable_pin, OUTPUT);
    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    pinMode(config::calcium_cam_status_pin, INPUT_PULLDOWN);
    pinMode(config::fiducial_cam_status_pin, INPUT_PULLDOWN);
    // Use the largest PSRAM block available for the samples.
    max_n_sample_bytes = ESP.getMaxAllocPsram();
    sample_bytes = static_cast<uint8_t *>(ps_malloc(max_n_sample_bytes));
    if (sample_bytes == nullptr) {
        max_n_sample_bytes = 0;
    }
    Serial.begin(config::serial_baud_rate);
    show_title();
}

void loop() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            command[command_length] = '\0';
            handle_command();
            command_length = 0;
        } else if (command_length < config::command_buffer_size - 1) {
            command[command_length++] = c;
        }
    }
}
