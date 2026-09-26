// Measures the status (SMA #4, common time) edges of both PCO muscle cameras
// after releasing their shared acquire-enable line, and sends the edge times
// to the host. See ../README.md.
//
// Serial protocol: the host sends "<shutter_open_time_us> <n_frames>\n" (e.g.
// "15147 5000\n"), and the firmware answers with a binary response (see
// config::response_magic).

#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include <cstdio>
#include <vector>

#include "pco_clock_drift/config.h"

// The edge times are sent as raw 4-byte values.
static_assert(sizeof(unsigned long) == sizeof(uint32_t));

namespace {
enum class Status : uint32_t {
    ok = 0,
    bad_command = 1,
    calcium_timeout = 2,
    fiducial_timeout = 3,
};

struct Camera {
    int status_pin;
    Status timeout_status;
    std::vector<unsigned long> on_edge_times_us;
    std::vector<unsigned long> off_edge_times_us;
};

Camera cameras[] = {
    {config::calcium_cam_status_pin, Status::calcium_timeout, {}, {}},
    {config::fiducial_cam_status_pin, Status::fiducial_timeout, {}, {}},
};
constexpr size_t n_cameras = sizeof(cameras) / sizeof(cameras[0]);

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

// Releases acquire enable and records the time of each camera's first
// n_frames rising and falling status edges. Busy-polls both pins, so each edge
// is timestamped to within one loop iteration (about a microsecond).
Status measure(unsigned long shutter_open_time_us, size_t n_frames) {
    for (Camera &camera : cameras) {
        camera.on_edge_times_us.assign(n_frames, 0);
        camera.off_edge_times_us.assign(n_frames, 0);
    }
    const unsigned long edge_timeout_us =
        config::edge_timeout_factor * shutter_open_time_us;

    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    delay(config::acquire_disable_time_ms);

    bool was_high[n_cameras];
    size_t n_on[n_cameras] = {};
    size_t n_off[n_cameras] = {};
    unsigned long last_edge_us[n_cameras];
    for (size_t i = 0; i < n_cameras; ++i) {
        was_high[i] = digitalRead(cameras[i].status_pin) == HIGH;
    }

    digitalWrite(config::musc_cam_acquire_enable_pin, HIGH);
    const unsigned long start_us = micros();
    for (unsigned long &t : last_edge_us) {
        t = start_us;
    }

    bool is_done = false;
    while (!is_done) {
        const unsigned long now_us = micros();
        is_done = true;
        for (size_t i = 0; i < n_cameras; ++i) {
            if (n_off[i] == n_frames) {
                continue;
            }
            is_done = false;
            const bool is_high = digitalRead(cameras[i].status_pin) == HIGH;
            if (is_high != was_high[i]) {
                const unsigned long t_us = now_us - start_us;
                if (is_high) {
                    cameras[i].on_edge_times_us[n_on[i]++] = t_us;
                } else if (n_off[i] < n_on[i]) {
                    // Falling edges before the first rising edge are ignored.
                    cameras[i].off_edge_times_us[n_off[i]++] = t_us;
                }
                was_high[i] = is_high;
                last_edge_us[i] = now_us;
            } else if (now_us - last_edge_us[i] > edge_timeout_us) {
                digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
                return cameras[i].timeout_status;
            }
        }
    }

    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    return Status::ok;
}

void send_response(Status status, size_t n_frames) {
    const uint32_t header[] = {
        config::response_magic,
        static_cast<uint32_t>(status),
        static_cast<uint32_t>(n_frames),
    };
    Serial.write(reinterpret_cast<const uint8_t *>(header), sizeof(header));
    if (status == Status::ok) {
        for (const Camera &camera : cameras) {
            for (const auto *times :
                 {&camera.on_edge_times_us, &camera.off_edge_times_us}) {
                Serial.write(
                    reinterpret_cast<const uint8_t *>(times->data()),
                    times->size() * sizeof(unsigned long));
            }
        }
    }
    Serial.flush();
}

void handle_command() {
    unsigned long shutter_open_time_us = 0;
    unsigned long n_frames = 0;
    const bool is_valid =
        std::sscanf(command, "%lu %lu", &shutter_open_time_us, &n_frames) ==
            2 &&
        shutter_open_time_us > 0 && n_frames >= 2 &&
        n_frames <= config::max_n_frames;
    if (!is_valid) {
        send_response(Status::bad_command, 0);
        return;
    }
    send_response(measure(shutter_open_time_us, n_frames), n_frames);
    for (Camera &camera : cameras) {
        // Free the buffers between measurements.
        std::vector<unsigned long>().swap(camera.on_edge_times_us);
        std::vector<unsigned long>().swap(camera.off_edge_times_us);
    }
}
} // namespace

void setup() {
    pinMode(config::musc_cam_acquire_enable_pin, OUTPUT);
    digitalWrite(config::musc_cam_acquire_enable_pin, LOW);
    for (const Camera &camera : cameras) {
        pinMode(camera.status_pin, INPUT_PULLDOWN);
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
