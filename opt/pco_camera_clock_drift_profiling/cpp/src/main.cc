// Records the status (SMA #4, common time) lines of the two free-running PCO
// muscle cameras after they are started on the same acquire-enable edge, to
// profile how far they drift apart. The Arduino samples both lines; this
// program only runs the cameras and saves the samples. The analysis is done in
// ../python/. See ../README.md.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// clang-format off
// stdafx.h must come first: it includes pco_linux_defs.h (WORD/BYTE/DWORD) and
// <climits>, which camera.h and cameraexception.h depend on.
#include "stdafx.h"
#include "camera.h"
#include "cameraexception.h"
// clang-format on

namespace {
namespace config {
constexpr DWORD calcium_serial_number = 14400755;
constexpr DWORD fiducial_serial_number = 14400057;

// Stable path: /dev/ttyACM* numbering depends on enumeration order (the Zaber
// stage controller is also a ttyACM device).
constexpr const char *serial_port =
    "/dev/serial/by-id/usb-Arduino_NanoESP32_48CA432E0DB4-if01";

// The Arduino's sampling loop (micros() and two digitalRead()s) takes about
// 1.45 us per sample, so the sample period must be at least 2 us. The PSRAM
// holds about 33M samples (8 MB, 4 samples per byte), i.e. about 4400 frames.
constexpr uint32_t n_frames = 1000;
constexpr uint32_t sample_period_us = 2;

// Operating point from docs/users_manual/frame_rate_limits.md
constexpr double frame_rate_hz = 66;
constexpr double line_time_us = 12.136;
constexpr double readout_time_us = 4.5;
constexpr unsigned int full_frame_size = 2048; // px, square sensor
constexpr unsigned int roi_size = 1120;        // px, centered square ROI

// The cameras must record for the exposures to run, but the images are never
// read, so a small ring buffer suffices.
constexpr DWORD recorder_buffer_size = 10;

// Extra time allowed for the Arduino's response header beyond the sampling
constexpr std::chrono::seconds response_timeout_margin{10};
// Lower bound on the USB transfer rate of the samples, for the read timeout
constexpr double min_transfer_rate_bytes_per_s = 100e3;
// Time acquire enable is held LOW before sampling (mirrors the firmware)
constexpr std::chrono::seconds acquire_disable_time{1};
// Response protocol (mirrors arduino/include/pco_clock_drift/config.h)
constexpr uint32_t response_magic = 0x54465244;
constexpr uint32_t samples_per_byte = 4;
} // namespace config

constexpr double frame_interval_us = 1e6 / config::frame_rate_hz;
constexpr double shutter_open_time_us =
    frame_interval_us - config::readout_time_us;
// One extra frame, since the first status pulse starts about one frame after
// acquire enable goes HIGH.
constexpr double sampling_duration_us = (config::n_frames + 1) * frame_interval_us;

// Response status codes (mirrors the firmware)
enum class ResponseStatus : uint32_t {
    ok = 0,
    bad_command = 1,
    buffer_too_small = 2,
};

std::string to_string(ResponseStatus status) {
    switch (status) {
    case ResponseStatus::ok:
        return "ok";
    case ResponseStatus::bad_command:
        return "bad command";
    case ResponseStatus::buffer_too_small:
        return "sample buffer too small (reduce n_frames)";
    }
    return std::format("unknown status {}", static_cast<uint32_t>(status));
}

// Raw serial port to the Arduino (native USB CDC, so the baud rate is moot).
class SerialPort {
  public:
    explicit SerialPort(const std::string &path)
        : fd_(open(path.c_str(), O_RDWR | O_NOCTTY)) {
        if (fd_ < 0) {
            throw std::runtime_error(std::format(
                "Cannot open serial port {}: {}", path, std::strerror(errno)));
        }
        termios tty{};
        tcgetattr(fd_, &tty);
        cfmakeraw(&tty);
        cfsetspeed(&tty, B115200);
        tcsetattr(fd_, TCSANOW, &tty);
        // Discard anything left over from a previous session.
        tcflush(fd_, TCIOFLUSH);
    }

    ~SerialPort() { close(fd_); }

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    void write_all(const std::string &data) {
        size_t n_written = 0;
        while (n_written < data.size()) {
            const ssize_t n =
                write(fd_, data.data() + n_written, data.size() - n_written);
            if (n < 0) {
                throw std::runtime_error(std::format(
                    "Serial write failed: {}", std::strerror(errno)));
            }
            n_written += n;
        }
    }

    void read_exact(
        void *buffer,
        size_t size,
        std::chrono::steady_clock::time_point deadline) {
        auto *bytes = static_cast<uint8_t *>(buffer);
        size_t n_read = 0;
        while (n_read < size) {
            const auto remaining =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now());
            pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
            if (remaining.count() <= 0 ||
                poll(&pfd, 1, static_cast<int>(remaining.count())) == 0) {
                throw std::runtime_error(std::format(
                    "Timed out waiting for the Arduino ({} of {} bytes read)",
                    n_read,
                    size));
            }
            const ssize_t n = read(fd_, bytes + n_read, size - n_read);
            if (n <= 0) {
                throw std::runtime_error(std::format(
                    "Serial read failed: {}", std::strerror(errno)));
            }
            n_read += n;
        }
    }

  private:
    int fd_;
};

void configure_camera(pco::Camera &camera) {
    camera.defaultConfiguration();
    pco::Configuration config = camera.getConfiguration();
    // PCO ROIs are 1-based and inclusive.
    const unsigned int roi_start =
        (config::full_frame_size - config::roi_size) / 2 + 1;
    const unsigned int roi_end = roi_start + config::roi_size - 1;
    config.roi.x0 = roi_start;
    config.roi.x1 = roi_end;
    config.roi.y0 = roi_start;
    config.roi.y1 = roi_end;
    // Free-running continuous rolling shutter, gated by acquire enable, as in
    // pco-camera-server (recorder/src/apps/pco_camera_server_main.cc).
    config.trigger_mode = TRIGGER_MODE_AUTOTRIGGER;
    if (!camera.getDescription().has_acquire_mode) {
        throw std::runtime_error("PCO camera does not support acquire modes");
    }
    config.acquire_mode = ACQUIRE_MODE_EXTERNAL;
    config.delay_time_s = 0;
    camera.setConfiguration(config);

    camera.setExposureTime(shutter_open_time_us / 1e6);
    camera.autoExposureOff();

    // Acquire enable on HWIO 2 (HIGH = enabled), common time on SMA #4.
    camera.configureHWIO_2_acquireEnable(true, pco::HWIO_Polarity::high_level);
    camera.configureHWIO_4_statusExpos(
        true,
        pco::HWIO_Polarity::high_level,
        pco::HWIO_4_SignalType::status_expos,
        pco::HWIO_StatusExpos_Timing::global);
}

void check_serial_number(pco::Camera &camera, DWORD expected) {
    const DWORD actual = camera.getDescription().serial;
    if (actual != expected) {
        throw std::runtime_error(std::format(
            "Opened PCO camera with serial number {}, expected {}",
            actual,
            expected));
    }
}

// Asks the Arduino to release acquire enable and sample both status lines, and
// returns the packed samples (see the protocol in config.h) and their number.
// Both sides are little-endian.
std::vector<uint8_t> sample(SerialPort &port, uint32_t &n_samples) {
    port.write_all(std::format(
        "{} {}\n",
        std::lround(sampling_duration_us),
        config::sample_period_us));

    const auto header_deadline =
        std::chrono::steady_clock::now() + config::acquire_disable_time +
        std::chrono::microseconds(std::lround(sampling_duration_us)) +
        config::response_timeout_margin;
    uint32_t header[3];
    port.read_exact(header, sizeof(header), header_deadline);
    const auto [magic, status, n] = header;
    if (magic != config::response_magic) {
        throw std::runtime_error(
            std::format("Unexpected response magic 0x{:08x}", magic));
    }
    if (static_cast<ResponseStatus>(status) != ResponseStatus::ok) {
        throw std::runtime_error(std::format(
            "Arduino reported an error: {}",
            to_string(static_cast<ResponseStatus>(status))));
    }

    n_samples = n;
    std::vector<uint8_t> bytes(
        (n_samples + config::samples_per_byte - 1) / config::samples_per_byte);
    const auto data_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(std::lround(
            bytes.size() / config::min_transfer_rate_bytes_per_s));
    port.read_exact(bytes.data(), bytes.size(), data_deadline);
    return bytes;
}

// Saves the packed samples as received (samples.bin) and the metadata needed
// to interpret them (metadata.json) in output_dir.
void save(
    const std::vector<uint8_t> &bytes,
    uint32_t n_samples,
    double calcium_shutter_open_time_us,
    double fiducial_shutter_open_time_us,
    const std::filesystem::path &output_dir) {
    std::ofstream bin_file(output_dir / "samples.bin", std::ios::binary);
    std::ofstream json_file(output_dir / "metadata.json");
    if (!bin_file || !json_file) {
        throw std::runtime_error(
            std::format("Cannot write to {}", output_dir.string()));
    }
    bin_file.write(
        reinterpret_cast<const char *>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
    json_file << std::format(
        R"({{
  "n_samples": {},
  "sample_period_us": {},
  "n_frames": {},
  "frame_rate_hz": {},
  "line_time_us": {},
  "readout_time_us": {},
  "roi_size": {},
  "calcium_shutter_open_time_us": {},
  "fiducial_shutter_open_time_us": {}
}}
)",
        n_samples,
        config::sample_period_us,
        config::n_frames,
        config::frame_rate_hz,
        config::line_time_us,
        config::readout_time_us,
        config::roi_size,
        calcium_shutter_open_time_us,
        fiducial_shutter_open_time_us);
}

void run() {
    std::cout << "Opening PCO cameras\n";
    pco::Camera calcium(
        pco::CameraInterface::Any, config::calcium_serial_number);
    check_serial_number(calcium, config::calcium_serial_number);
    pco::Camera fiducial(
        pco::CameraInterface::Any, config::fiducial_serial_number);
    check_serial_number(fiducial, config::fiducial_serial_number);

    SerialPort port(config::serial_port);

    for (pco::Camera *camera : {&calcium, &fiducial}) {
        configure_camera(*camera);
        camera->record(
            config::recorder_buffer_size, pco::RecordMode::ring_buffer);
    }
    // Applied shutter-open times (the cameras round the requested value)
    const double calcium_shutter_open_time_us = calcium.getExposureTime() * 1e6;
    const double fiducial_shutter_open_time_us =
        fiducial.getExposureTime() * 1e6;

    std::cout << std::format(
                     "Sampling {} frames every {} us",
                     config::n_frames,
                     config::sample_period_us)
              << '\n';
    uint32_t n_samples = 0;
    const std::vector<uint8_t> bytes = sample(port, n_samples);
    calcium.stop();
    fiducial.stop();

    const auto now = std::chrono::floor<std::chrono::seconds>(
        std::chrono::system_clock::now());
    const std::filesystem::path output_dir = std::format(
        "clock_drift_{:%Y%m%d_%H%M%S}",
        std::chrono::zoned_time(std::chrono::current_zone(), now));
    std::filesystem::create_directories(output_dir);
    save(
        bytes,
        n_samples,
        calcium_shutter_open_time_us,
        fiducial_shutter_open_time_us,
        output_dir);
    std::cout << std::format("Saved {}", output_dir.string()) << '\n';
}
} // namespace

int main() {
    // The PCO SDK must be initialized before any pco::Camera is constructed.
    if (const int err = PCO_InitializeLib(); err != PCO_NOERROR) {
        std::cerr << std::format(
                         "Failed to initialize PCO SDK (error 0x{:08x})",
                         static_cast<uint32_t>(err))
                  << '\n';
        return 1;
    }
    int exit_code = 0;
    try {
        run();
    } catch (pco::CameraException &e) {
        std::cerr << std::format(
                         "PCO camera error (0x{:08x}): {}",
                         static_cast<uint32_t>(e.error_code()),
                         e.what())
                  << '\n';
        exit_code = 1;
    } catch (const std::exception &e) {
        std::cerr << std::format("Error: {}", e.what()) << '\n';
        exit_code = 1;
    }
    PCO_CleanupLib();
    return exit_code;
}
