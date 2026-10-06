// Tests whether the two free-running PCO muscle cameras can be re-synced with
// the acquire-enable line while they are recording and their frames are being
// grabbed, without stopping and restarting the recording in software. The
// Arduino periodically holds acquire enable LOW (as trigger_firmware does when
// it restarts its timing) and samples both status lines; this program grabs
// both cameras' frames as pco-camera-server does and saves the samples and the
// grab logs. The analysis is done in ../python/. See ../README.md.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
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
#include <thread>
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

// The Arduino's sampling loop takes about 1.45 us per sample, so the sample
// period must be at least 2 us. The PSRAM holds about 33M samples (8 MB, 4
// samples per byte), i.e. about 66 s at 2 us.
constexpr uint32_t sample_period_us = 2;
constexpr std::chrono::seconds sampling_duration{40};
// Acquire enable is held LOW every resync_interval for one frame interval plus
// acquire_restart_margin (as trigger_firmware's
// config::musc_acquire_restart_margin_us).
constexpr std::chrono::seconds resync_interval{10};
constexpr double acquire_restart_margin_us = 2000;

// Operating point from docs/users_manual/frame_rate_limits.md
constexpr double frame_rate_hz = 66;
constexpr double line_time_us = 12.136;
constexpr double readout_time_us = 4.5;
constexpr unsigned int full_frame_size = 2048; // px, square sensor
constexpr unsigned int roi_size = 1120;        // px, centered square ROI

// Same as pco-camera-server (recorder/src/apps/pco_camera_server_main.cc and
// recorder/include/recorder/apps/pco_camera_server.h)
constexpr DWORD recorder_buffer_size = 10;
constexpr bool wait_with_small_delay = true;
constexpr double wait_timeout_s = 0.1;
constexpr uint32_t timeout_error_code = 0x80004001;

// Extra time allowed for the Arduino's response header beyond the sampling
constexpr std::chrono::seconds response_timeout_margin{10};
// Lower bound on the USB transfer rate of the samples, for the read timeout
constexpr double min_transfer_rate_bytes_per_s = 100e3;
// Time acquire enable is held LOW before sampling (mirrors the firmware)
constexpr std::chrono::seconds acquire_disable_time{1};
// Response protocol (mirrors arduino/include/pco_resync_test/config.h)
constexpr uint32_t response_magic = 0x434e5352;
constexpr uint32_t samples_per_byte = 4;
} // namespace config

constexpr double frame_interval_us = 1e6 / config::frame_rate_hz;
constexpr double shutter_open_time_us =
    frame_interval_us - config::readout_time_us;
constexpr double acquire_low_us =
    frame_interval_us + config::acquire_restart_margin_us;

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
        return "sample buffer too small (reduce sampling_duration)";
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

// Configures the camera as pco-camera-server's setup_pco_camera() does.
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
    // Free-running continuous rolling shutter, gated by acquire enable
    config.trigger_mode = TRIGGER_MODE_AUTOTRIGGER;
    const pco::Description description = camera.getDescription();
    if (!description.has_acquire_mode) {
        throw std::runtime_error("PCO camera does not support acquire modes");
    }
    config.acquire_mode = ACQUIRE_MODE_EXTERNAL;
    config.delay_time_s = 0;
    config.noise_filter_mode = NOISE_FILTER_MODE_ON;
    // Unlike pco-camera-server, no binary timestamps: these cameras only
    // support ASCII timestamps (GENERALCAPS1_TIMESTAMP_ASCII_ONLY).
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

struct GrabbedFrame {
    int64_t host_time_us; // since the sampling command was sent
    DWORD recorder_image_number;
};

struct GrabError {
    int64_t host_time_us;
    uint32_t error_code;
    std::string message;
};

struct GrabLog {
    std::vector<GrabbedFrame> frames;
    std::vector<GrabError> errors;
};

// Grabs frames as pco-camera-server's serve_frames() does (wait for a new
// image with a short timeout, then fetch the latest image) until stop is set.
// Errors other than timeouts are logged, not thrown, so that the test shows
// whether grabbing recovers after a re-sync.
void grab_frames(
    pco::Camera &camera,
    std::chrono::steady_clock::time_point start_time,
    const std::atomic<bool> &stop,
    GrabLog &log) {
    const auto elapsed_us = [&] {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - start_time)
            .count();
    };
    pco::Image image;
    bool is_first_frame = true;
    while (!stop.load()) {
        try {
            if (is_first_frame) {
                camera.waitForFirstImage(
                    config::wait_with_small_delay, config::wait_timeout_s);
                is_first_frame = false;
            } else {
                camera.waitForNewImage(
                    config::wait_with_small_delay, config::wait_timeout_s);
            }
            camera.image(
                image, PCO_RECORDER_LATEST_IMAGE, pco::DataFormat::Mono16);
        } catch (pco::CameraException &e) {
            const auto error_code = static_cast<uint32_t>(e.error_code());
            if (error_code != config::timeout_error_code) {
                log.errors.push_back(
                    {.host_time_us = elapsed_us(),
                     .error_code = error_code,
                     .message = e.what()});
            }
            continue;
        }
        log.frames.push_back(
            {.host_time_us = elapsed_us(),
             .recorder_image_number = image.getRecorderImageNumber()});
    }
}

// Asks the Arduino to release acquire enable, sample both status lines, and
// re-sync periodically, and returns the packed samples (see the protocol in
// config.h) and their number. Both sides are little-endian.
std::vector<uint8_t> sample(SerialPort &port, uint32_t &n_samples) {
    const auto duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            config::sampling_duration);
    port.write_all(std::format(
        "{} {} {} {}\n",
        duration_us.count(),
        config::sample_period_us,
        std::chrono::duration_cast<std::chrono::microseconds>(
            config::resync_interval)
            .count(),
        std::lround(acquire_low_us)));

    const auto header_deadline = std::chrono::steady_clock::now() +
                                 config::acquire_disable_time + duration_us +
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

// Saves the packed samples as received (samples.bin), the metadata needed to
// interpret them (metadata.json), the grabbed frames (frames_<camera>.csv),
// and the grab errors (grab_errors.csv) in output_dir.
void save(
    const std::vector<uint8_t> &bytes,
    uint32_t n_samples,
    double calcium_shutter_open_time_us,
    double fiducial_shutter_open_time_us,
    const GrabLog &calcium_log,
    const GrabLog &fiducial_log,
    const std::filesystem::path &output_dir) {
    std::ofstream bin_file(output_dir / "samples.bin", std::ios::binary);
    std::ofstream json_file(output_dir / "metadata.json");
    std::ofstream errors_file(output_dir / "grab_errors.csv");
    if (!bin_file || !json_file || !errors_file) {
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
  "resync_interval_us": {},
  "acquire_low_us": {},
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
        std::chrono::duration_cast<std::chrono::microseconds>(
            config::resync_interval)
            .count(),
        std::lround(acquire_low_us),
        config::frame_rate_hz,
        config::line_time_us,
        config::readout_time_us,
        config::roi_size,
        calcium_shutter_open_time_us,
        fiducial_shutter_open_time_us);

    errors_file << "camera,host_time_us,error_code,message\n";
    for (const auto &[name, log] :
         {std::pair{"calcium", &calcium_log},
          std::pair{"fiducial", &fiducial_log}}) {
        std::ofstream frames_file(
            output_dir / std::format("frames_{}.csv", name));
        frames_file << "host_time_us,recorder_image_number\n";
        for (const GrabbedFrame &frame : log->frames) {
            frames_file << std::format(
                "{},{}\n", frame.host_time_us, frame.recorder_image_number);
        }
        // Commas and quotes are dropped from the messages to keep the CSV
        // simple.
        for (const GrabError &error : log->errors) {
            std::string message = error.message;
            std::erase_if(message, [](char c) { return c == ',' || c == '"'; });
            errors_file << std::format(
                "{},{},0x{:08x},{}\n",
                name,
                error.host_time_us,
                error.error_code,
                message);
        }
    }
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
                     "Grabbing and sampling for {}, re-syncing every {}",
                     config::sampling_duration,
                     config::resync_interval)
              << '\n';
    const auto start_time = std::chrono::steady_clock::now();
    std::atomic<bool> stop = false;
    GrabLog calcium_log;
    GrabLog fiducial_log;
    std::thread calcium_thread(
        grab_frames,
        std::ref(calcium),
        start_time,
        std::cref(stop),
        std::ref(calcium_log));
    std::thread fiducial_thread(
        grab_frames,
        std::ref(fiducial),
        start_time,
        std::cref(stop),
        std::ref(fiducial_log));
    uint32_t n_samples = 0;
    std::vector<uint8_t> bytes;
    try {
        bytes = sample(port, n_samples);
    } catch (...) {
        stop.store(true);
        calcium_thread.join();
        fiducial_thread.join();
        throw;
    }
    stop.store(true);
    calcium_thread.join();
    fiducial_thread.join();
    calcium.stop();
    fiducial.stop();

    const auto now = std::chrono::floor<std::chrono::seconds>(
        std::chrono::system_clock::now());
    const std::filesystem::path output_dir = std::format(
        "resync_test_{:%Y%m%d_%H%M%S}",
        std::chrono::zoned_time(std::chrono::current_zone(), now));
    std::filesystem::create_directories(output_dir);
    save(
        bytes,
        n_samples,
        calcium_shutter_open_time_us,
        fiducial_shutter_open_time_us,
        calcium_log,
        fiducial_log,
        output_dir);
    std::cout << std::format(
                     "Grabbed {} calcium and {} fiducial frames ({} and {} "
                     "grab errors). Saved {}",
                     calcium_log.frames.size(),
                     fiducial_log.frames.size(),
                     calcium_log.errors.size(),
                     fiducial_log.errors.size(),
                     output_dir.string())
              << '\n';
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
