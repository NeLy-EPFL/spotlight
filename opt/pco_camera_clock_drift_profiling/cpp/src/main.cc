// Profiles how far the two free-running PCO muscle cameras drift apart after
// being started on the same acquire-enable edge. See ../README.md.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <print>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera.h"
#include "cameraexception.h"
#include "stdafx.h"

namespace {
namespace config {
// Camera serial numbers (placeholders: set to the real serial numbers)
constexpr DWORD calcium_serial_number = 0;
constexpr DWORD fiducial_serial_number = 0;

constexpr const char *serial_port = "/dev/ttyACM0";

constexpr int n_repeats = 5;
constexpr uint32_t n_frames = 5000;

// Operating point from docs/users_manual/frame_rate_limits.md
constexpr double frame_rate_hz = 66;
constexpr double line_time_us = 12.136;
constexpr double readout_time_us = 4.5;
constexpr unsigned int full_frame_size = 2048; // px, square sensor
constexpr unsigned int roi_size = 1120;        // px, centered square ROI

// The cameras must record for the exposures to run, but the images are never
// read, so a small ring buffer suffices.
constexpr DWORD recorder_buffer_size = 10;

// Extra time allowed for the firmware's response beyond the measurement itself
constexpr std::chrono::seconds response_timeout_margin{10};
// Time acquire enable is held LOW before a measurement (mirrors the firmware)
constexpr std::chrono::seconds acquire_disable_time{1};
// Response header magic number (mirrors the firmware)
constexpr uint32_t response_magic = 0x54465244;
} // namespace config

// Timing derived from the operating point (see frame_rate_limits.md)
constexpr double frame_interval_us = 1e6 / config::frame_rate_hz;
constexpr double rolling_time_us = config::roi_size * config::line_time_us;
constexpr double shutter_open_time_us =
    frame_interval_us - config::readout_time_us;
constexpr double common_time_us = shutter_open_time_us - rolling_time_us;

// Response status codes (mirrors the firmware)
enum class ResponseStatus : uint32_t {
    ok = 0,
    bad_command = 1,
    calcium_timeout = 2,
    fiducial_timeout = 3,
};

std::string to_string(ResponseStatus status) {
    switch (status) {
    case ResponseStatus::ok:
        return "ok";
    case ResponseStatus::bad_command:
        return "bad command";
    case ResponseStatus::calcium_timeout:
        return "calcium camera status timeout";
    case ResponseStatus::fiducial_timeout:
        return "fiducial camera status timeout";
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

// Status edge times of one camera, in us since acquire enable went HIGH.
struct CameraEdges {
    std::vector<uint32_t> on_us;
    std::vector<uint32_t> off_us;
};

struct Measurement {
    CameraEdges calcium;
    CameraEdges fiducial;
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

// Asks the Arduino to release acquire enable and record the status edges of
// both cameras, then reads its binary response. Both sides are little-endian.
Measurement measure(SerialPort &port) {
    port.write_all(std::format(
        "{} {}\n", std::lround(shutter_open_time_us), config::n_frames));

    const auto measurement_duration = std::chrono::microseconds(
        std::lround(config::n_frames * frame_interval_us));
    const auto deadline = std::chrono::steady_clock::now() +
                          config::acquire_disable_time + measurement_duration +
                          config::response_timeout_margin;

    uint32_t header[3];
    port.read_exact(header, sizeof(header), deadline);
    const auto [magic, status, n_frames] = header;
    if (magic != config::response_magic) {
        throw std::runtime_error(
            std::format("Unexpected response magic 0x{:08x}", magic));
    }
    if (static_cast<ResponseStatus>(status) != ResponseStatus::ok) {
        throw std::runtime_error(std::format(
            "Arduino reported an error: {}",
            to_string(static_cast<ResponseStatus>(status))));
    }
    if (n_frames != config::n_frames) {
        throw std::runtime_error(std::format(
            "Arduino measured {} frames, expected {}",
            n_frames,
            config::n_frames));
    }

    Measurement measurement;
    for (std::vector<uint32_t> *times :
         {&measurement.calcium.on_us,
          &measurement.calcium.off_us,
          &measurement.fiducial.on_us,
          &measurement.fiducial.off_us}) {
        times->resize(n_frames);
        port.read_exact(times->data(), n_frames * sizeof(uint32_t), deadline);
    }
    return measurement;
}

// Duration between consecutive rising edges (one fewer than the frames).
std::vector<double> intervals_us(const CameraEdges &edges) {
    std::vector<double> intervals;
    for (size_t i = 0; i + 1 < edges.on_us.size(); ++i) {
        intervals.push_back(
            static_cast<double>(edges.on_us[i + 1]) - edges.on_us[i]);
    }
    return intervals;
}

std::vector<double> common_times_us(const CameraEdges &edges) {
    std::vector<double> durations;
    for (size_t i = 0; i < edges.on_us.size(); ++i) {
        durations.push_back(
            static_cast<double>(edges.off_us[i]) - edges.on_us[i]);
    }
    return durations;
}

struct DeviationStats {
    double mean_us;    // signed
    double max_abs_us; // largest deviation in either direction
};

DeviationStats
deviation_stats(const std::vector<double> &values_us, double expected_us) {
    double sum_us = 0;
    double max_abs_us = 0;
    for (const double value_us : values_us) {
        const double deviation_us = value_us - expected_us;
        sum_us += deviation_us;
        max_abs_us = std::max(max_abs_us, std::abs(deviation_us));
    }
    return {sum_us / values_us.size(), max_abs_us};
}

void print_camera_summary(const std::string &name, const CameraEdges &edges) {
    const DeviationStats interval =
        deviation_stats(intervals_us(edges), frame_interval_us);
    const DeviationStats common_time =
        deviation_stats(common_times_us(edges), common_time_us);
    std::println(
        "  {:<8} interval:    mean dev {:+8.3f} us ({:+.1f} ppm), "
        "max |dev| {:.3f} us",
        name,
        interval.mean_us,
        interval.mean_us / frame_interval_us * 1e6,
        interval.max_abs_us);
    std::println(
        "  {:<8} common time: mean dev {:+8.3f} us, max |dev| {:.3f} us",
        name,
        common_time.mean_us,
        common_time.max_abs_us);
}

void print_summary(const Measurement &measurement) {
    print_camera_summary("calcium", measurement.calcium);
    print_camera_summary("fiducial", measurement.fiducial);

    const auto &calcium_on_us = measurement.calcium.on_us;
    const auto &fiducial_on_us = measurement.fiducial.on_us;
    const double first_offset_us =
        static_cast<double>(fiducial_on_us.front()) - calcium_on_us.front();
    const double last_offset_us =
        static_cast<double>(fiducial_on_us.back()) - calcium_on_us.back();
    const double drift_us = last_offset_us - first_offset_us;
    const double elapsed_us =
        static_cast<double>(calcium_on_us.back()) - calcium_on_us.front();
    std::println(
        "  fiducial - calcium onset: first {:+.0f} us, last {:+.0f} us, "
        "drift {:+.0f} us over {:.1f} s ({:+.2f} ppm)",
        first_offset_us,
        last_offset_us,
        drift_us,
        elapsed_us / 1e6,
        drift_us / elapsed_us * 1e6);
}

// One row per frame: raw edge times plus the derived durations. A frame's
// interval is the time to the next frame's onset (empty for the last frame).
void write_csv(
    const Measurement &measurement,
    const std::filesystem::path &path) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error(
            std::format("Cannot open {} for writing", path.string()));
    }
    const CameraEdges &calcium = measurement.calcium;
    const CameraEdges &fiducial = measurement.fiducial;
    const std::vector<double> calcium_intervals = intervals_us(calcium);
    const std::vector<double> fiducial_intervals = intervals_us(fiducial);
    const std::vector<double> calcium_common_times = common_times_us(calcium);
    const std::vector<double> fiducial_common_times = common_times_us(fiducial);

    std::println(
        file,
        "frame,calcium_on_us,calcium_off_us,fiducial_on_us,fiducial_off_us,"
        "calcium_interval_us,fiducial_interval_us,calcium_common_time_us,"
        "fiducial_common_time_us,fiducial_minus_calcium_on_us");
    for (size_t i = 0; i < calcium.on_us.size(); ++i) {
        const bool has_interval = i < calcium_intervals.size();
        std::println(
            file,
            "{},{},{},{},{},{},{},{},{},{}",
            i,
            calcium.on_us[i],
            calcium.off_us[i],
            fiducial.on_us[i],
            fiducial.off_us[i],
            has_interval ? std::format("{}", calcium_intervals[i]) : "",
            has_interval ? std::format("{}", fiducial_intervals[i]) : "",
            calcium_common_times[i],
            fiducial_common_times[i],
            static_cast<double>(fiducial.on_us[i]) - calcium.on_us[i]);
    }
}

void run() {
    std::println(
        "Expected timing: frame interval {:.3f} us, shutter-open time {:.3f} "
        "us, common time {:.3f} us",
        frame_interval_us,
        shutter_open_time_us,
        common_time_us);

    std::println("Opening PCO cameras");
    pco::Camera calcium(
        pco::CameraInterface::Any, config::calcium_serial_number);
    check_serial_number(calcium, config::calcium_serial_number);
    pco::Camera fiducial(
        pco::CameraInterface::Any, config::fiducial_serial_number);
    check_serial_number(fiducial, config::fiducial_serial_number);

    SerialPort port(config::serial_port);

    const auto now = std::chrono::floor<std::chrono::seconds>(
        std::chrono::system_clock::now());
    const std::filesystem::path output_dir = std::format(
        "clock_drift_{:%Y%m%d_%H%M%S}",
        std::chrono::zoned_time(std::chrono::current_zone(), now));
    std::filesystem::create_directories(output_dir);

    for (int repeat = 1; repeat <= config::n_repeats; ++repeat) {
        std::println("Repeat {}/{}", repeat, config::n_repeats);
        for (pco::Camera *camera : {&calcium, &fiducial}) {
            configure_camera(*camera);
            std::println(
                "  camera {}: applied shutter-open time {:.3f} us",
                camera->getDescription().serial,
                camera->getExposureTime() * 1e6);
            camera->record(
                config::recorder_buffer_size, pco::RecordMode::ring_buffer);
        }

        std::println("  measuring {} frames", config::n_frames);
        const Measurement measurement = measure(port);
        calcium.stop();
        fiducial.stop();

        const std::filesystem::path csv_path =
            output_dir / std::format("repeat_{}.csv", repeat);
        write_csv(measurement, csv_path);
        print_summary(measurement);
        std::println("  saved {}", csv_path.string());
    }
}
} // namespace

int main() {
    // The PCO SDK must be initialized before any pco::Camera is constructed.
    if (const int err = PCO_InitializeLib(); err != PCO_NOERROR) {
        std::println(
            stderr,
            "Failed to initialize PCO SDK (error 0x{:08x})",
            static_cast<uint32_t>(err));
        return 1;
    }
    int exit_code = 0;
    try {
        run();
    } catch (pco::CameraException &e) {
        std::println(
            stderr,
            "PCO camera error (0x{:08x}): {}",
            static_cast<uint32_t>(e.error_code()),
            e.what());
        exit_code = 1;
    } catch (const std::exception &e) {
        std::println(stderr, "Error: {}", e.what());
        exit_code = 1;
    }
    PCO_CleanupLib();
    return exit_code;
}
