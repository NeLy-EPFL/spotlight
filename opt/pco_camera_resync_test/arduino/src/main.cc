// Samples the status (SMA #4, common time) lines of both PCO muscle cameras at
// a fixed rate after releasing their shared acquire-enable line, and
// periodically re-syncs the cameras by holding acquire enable LOW, as
// trigger_firmware does when it restarts its timing. Sends the whole sampled
// sequence to the host. Like trigger_firmware, it uses a plain loop with
// micros(), digitalRead(), and digitalWrite(). See ../README.md.
//
// Serial protocol: the host sends
// "<duration_us> <sample_period_us> <resync_interval_us> <acquire_low_us>\n"
// (e.g. "40000000 2 10000000 17147\n"), and the firmware answers with a binary
// response (see config::response_magic). Acquire enable goes LOW at every
// multiple of resync_interval_us after sampling starts, for acquire_low_us.

#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include <cstdio>

#include "pco_resync_test/config.h"

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
    display.println("RESYNC");
    display.println("TEST");
    display.display();
}

// Releases acquire enable and samples both status lines n_samples times, every
// sample_period_us. Every resync_interval_samples samples, holds acquire
// enable LOW for acquire_low_samples samples.
void sample(
    uint32_t n_samples,
    unsigned long sample_period_us,
    uint32_t resync_interval_samples,
    uint32_t acquire_low_samples) {
    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    delay(config::acquire_disable_time_ms);
    digitalWrite(config::musc_cam_acquire_enable_pin, HIGH);

    unsigned long next_sample_us = micros();
    uint8_t byte = 0;
    for (uint32_t i = 0; i < n_samples; ++i) {
        while (static_cast<long>(micros() - next_sample_us) < 0) {
        }
        next_sample_us += sample_period_us;
        const uint32_t phase = i % resync_interval_samples;
        if (i >= resync_interval_samples) {
            if (phase == 0) {
                digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
            } else if (phase == acquire_low_samples) {
                digitalWrite(config::musc_cam_acquire_enable_pin, HIGH);
            }
        }
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
    unsigned long resync_interval_us = 0;
    unsigned long acquire_low_us = 0;
    const bool is_valid =
        std::sscanf(
            command,
            "%lu %lu %lu %lu",
            &duration_us,
            &sample_period_us,
            &resync_interval_us,
            &acquire_low_us) == 4 &&
        duration_us > 0 && sample_period_us > 0 &&
        acquire_low_us < resync_interval_us;
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
    sample(
        n_samples,
        sample_period_us,
        /*resync_interval_samples=*/resync_interval_us / sample_period_us,
        /*acquire_low_samples=*/acquire_low_us / sample_period_us);
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
