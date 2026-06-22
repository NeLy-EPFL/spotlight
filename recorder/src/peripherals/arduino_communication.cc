#include "recorder/peripherals/arduino_communication.h"

namespace {
// Read whatever bytes the controller has sent and log each complete (newline-
// terminated) line at info level. `buffer` carries any partial trailing line
// across calls, so a message split across reads is reassembled. Waits up to 1
// ms for bytes to arrive, then returns; safe to call when nothing is pending.
void drain_incoming_serial(QSerialPort &serial_port, std::string &buffer) {
    if (!serial_port.waitForReadyRead(1)) {
        return;
    }
    QByteArray response_data = serial_port.readAll();
    if (response_data.isEmpty()) {
        return;
    }
    buffer += std::string(response_data.constData(), response_data.length());
    size_t newline_pos;
    while ((newline_pos = buffer.find('\n')) != std::string::npos) {
        std::string complete_line = buffer.substr(0, newline_pos);
        buffer.erase(0, newline_pos + 1);
        spdlog::info("Arduino said: {}", complete_line);
    }
}

// After a RESET command, the firmware calls esp_restart() and the controller
// reboots, which tears down and re-creates its native USB CDC serial port. Wait
// for it to come back, then reopen the (stable) port so that subsequent
// commands reach the rebooted controller. Reopening relies on a stable port
// name -- the /dev/arduino_trigger udev symlink -- surviving the
// re-enumeration.
void wait_for_controller_reboot(
    QSerialPort &serial_port,
    const std::string &port_name,
    std::atomic<bool> &stop_communication) {
    // Time for the MCU to reboot and its USB CDC port to re-enumerate before we
    // try to reopen it; generous so the first reopen attempt below succeeds.
    constexpr auto reboot_wait = std::chrono::milliseconds(2000);
    constexpr auto reopen_interval = std::chrono::milliseconds(200);
    constexpr int max_reopen_attempts = 50; // ~10 s total before giving up

    spdlog::info(
        "Trigger controller resetting (esp_restart); waiting for it to reboot "
        "and re-enumerate on {}",
        port_name);
    serial_port.close();
    std::this_thread::sleep_for(reboot_wait);

    for (int attempt = 0; attempt < max_reopen_attempts; ++attempt) {
        if (stop_communication) {
            return;
        }
        if (serial_port.open(QIODevice::ReadWrite)) {
            spdlog::info("Reconnected to trigger controller after reset");
            return;
        }
        std::this_thread::sleep_for(reopen_interval);
    }
    spdlog::error(
        "Trigger controller did not re-enumerate within timeout after reset; "
        "will keep retrying on the next command write");
}

void arduino_comm_thread_func(
    const std::string &port_name,
    int baud_rate,
    std::atomic<bool> &stop_communication,
    std::mutex &mutex,
    std::condition_variable &cv,
    std::queue<std::string> &arduino_messages_queue) {
    QSerialPort serial_port = QSerialPort();
    serial_port.setPortName(port_name.c_str());
    serial_port.setBaudRate(baud_rate);
    serial_port.setDataBits(QSerialPort::Data8);
    serial_port.setParity(QSerialPort::NoParity);
    serial_port.setStopBits(QSerialPort::OneStop);
    serial_port.setFlowControl(QSerialPort::NoFlowControl);

    spdlog::info("Opening Arduino comm on serial port {}", port_name);
    if (!serial_port.open(QIODevice::ReadWrite)) {
        spdlog::error("Failed to open serial port");
    }

    // Reset Arduino
    serial_port.setDataTerminalReady(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial_port.setDataTerminalReady(true);

    std::string incoming_message_buffer;

    // The controller frames commands by newline; reset() enqueues the RESET
    // command the same way, so it can be recognized here by its exact
    // serialized form to drive the post-reboot reconnect (see
    // wait_for_controller_reboot).
    const std::string reset_command_line =
        Command::make_reset_command().to_string() + "\n";

    while (true) {
        std::string message;
        bool has_message = false;
        {
            std::unique_lock<std::mutex> lock(mutex);
            // Wake on a new outgoing command or on shutdown, but also time out
            // periodically so incoming bytes are drained even while idle (the
            // host sends nothing during steady-state streaming/recording). A
            // notify still returns immediately, so command sending stays
            // prompt.
            cv.wait_for(lock, std::chrono::milliseconds(10), [&] {
                return !arduino_messages_queue.empty() || stop_communication;
            });
            if (stop_communication) {
                break;
            }
            if (!arduino_messages_queue.empty()) {
                message = arduino_messages_queue.front();
                arduino_messages_queue.pop();
                has_message = true;
            }
        }

        // Send the outgoing command, if any. On repeated write failures, try
        // reconnecting to the serial port.
        if (has_message) {
            int consecutive_failed_writes = 0;
            int max_consecutive_failed_writes = 2;
            while (true) {
                serial_port.write(message.c_str());
                if (!serial_port.waitForBytesWritten(1000)) {
                    spdlog::error(
                        "Failed to write message: {} to serial port", message);
                    consecutive_failed_writes++;
                    if (consecutive_failed_writes >=
                        max_consecutive_failed_writes) {
                        spdlog::error("Max consecutive failed writes reached. "
                                      "Reconnecting...");
                        serial_port.close();
                        if (!serial_port.open(QIODevice::ReadWrite)) {
                            spdlog::error("Failed to reopen serial port. "
                                          "Stopping Arduino "
                                          "communication thread.");
                            break;
                        }
                        consecutive_failed_writes = 0;
                        spdlog::info(
                            "Successfully reconnected to serial port.");
                    }
                    continue;
                }
                consecutive_failed_writes = 0;
                break;
            }

            // A RESET reboots the controller and drops the serial link; wait
            // for it to come back and reopen the port before sending anything
            // else.
            if (message == reset_command_line) {
                wait_for_controller_reboot(
                    serial_port, port_name, stop_communication);
            }
        }

        // Drain incoming bytes every iteration -- after a send and on idle
        // wake-ups -- so the controller's USB-CDC TX buffer never backs up. A
        // full TX buffer can block Serial writes inside the firmware's timing
        // loop and stall triggering, so prompt draining protects timing.
        drain_incoming_serial(serial_port, incoming_message_buffer);
    }

    if (serial_port.isOpen()) {
        serial_port.close();
    }
}
} // namespace

ArduinoCommunication::ArduinoCommunication(
    const std::string &port_name, int baud_rate) {
    arduino_comm_thread_ = std::thread(
        arduino_comm_thread_func,
        port_name,
        baud_rate,
        std::ref(stop_communication_),
        std::ref(mutex_),
        std::ref(cv_),
        std::ref(arduino_messages_queue_));
}

ArduinoCommunication::~ArduinoCommunication() {
    // Always signal the comm thread to stop before joining it. Without this, an
    // owner that forgets to call stop_communication() (as the run-spotlight
    // shutdown path does) would deadlock here forever, since the thread's loop
    // only exits once stop_communication_ is set. stop_communication() is
    // idempotent, so this is harmless when the owner already called it.
    stop_communication();
    if (arduino_comm_thread_.joinable()) {
        arduino_comm_thread_.join();
    }
}

void ArduinoCommunication::enqueue_message(const std::string &message) {
    // The controller frames commands by newline (see trigger_firmware
    // SerialIO), so every JSON command is terminated with one.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduino_messages_queue_.push(message + "\n");
    }
    cv_.notify_one();
}

void ArduinoCommunication::stream(const TriggerParams &params) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_stream_params_ = params;
    }
    enqueue_message(Command::make_stream_command(params).to_string());
}

void ArduinoCommunication::start_recording(
    const TriggerParams &rec_params,
    const TriggerParams &revert_to_params,
    const std::deque<OperationStep> &op_sequence) {
    enqueue_message(Command::make_start_recording_command(
                        rec_params, revert_to_params, op_sequence)
                        .to_string());
}

void ArduinoCommunication::stop_recording() {
    enqueue_message(Command::make_stop_recording_command().to_string());
}

void ArduinoCommunication::reset() {
    // Goes through the same FIFO queue as every other command, so any command
    // enqueued afterwards (e.g. the initial STREAM) is sent only once the comm
    // thread has waited out the reboot and reopened the port.
    enqueue_message(Command::make_reset_command().to_string());
}

void ArduinoCommunication::stop_excitation() {
    TriggerParams params;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        params = last_stream_params_;
    }
    // Disable the muscle camera entirely: the controller free-runs the behavior
    // camera on its own clock and never pulses the blue excitation LED.
    params.enable_muscle = false;
    stream(params);
}

void ArduinoCommunication::stop_communication() {
    stop_communication_ = true;
    cv_.notify_one();
}

TriggerParams make_default_stream_params(
    const RecorderConfig &recorder_config,
    int muscle_num_lines_scanned,
    int sync_ratio,
    bool muscle_imaging_on) {
    TriggerParams params;
    params.beh_frame_rate = recorder_config.get_parameter<int>(
        "behavior_camera", "streaming_frame_rate");
    params.beh_exp_time = recorder_config.get_parameter<int>(
        "behavior_camera", "default_exposure_time_us");
    params.beh_musc_sync_ratio = sync_ratio >= 1 ? sync_ratio : 1;
    // enable_muscle selects the controller's mode; when false the muscle-only
    // fields below are sent but ignored (the behavior camera free-runs).
    params.enable_muscle = muscle_imaging_on;
    int muscle_light_on_time_us = recorder_config.get_parameter<int>(
        "muscle_camera", "default_light_on_time_us");
    params.musc_eff_exp_time = muscle_light_on_time_us;
    double rolling_shutter_line_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensor_readout_time_us = recorder_config.get_parameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    params.pco_cam_rolling_time = static_cast<unsigned int>(
        muscle_num_lines_scanned * rolling_shutter_line_time_us);
    params.pco_cam_readout_time =
        static_cast<unsigned int>(sensor_readout_time_us);
    return params;
}

std::string find_arduino_port_name(const RecorderConfig &recorder_config) {
    std::string udev_port_name = "/dev/arduino_trigger";
    if (fs::exists(udev_port_name)) {
        spdlog::info(
            "Serial port '{}' found via udev. Using it without further checks.",
            udev_port_name);
        return udev_port_name;
    } else {
        std::string arduino_manufacturer =
            recorder_config.get_parameter<std::string>(
                "triggering", "arduino_device_manufacturer");
        std::string arduino_description =
            recorder_config.get_parameter<std::string>(
                "triggering", "arduino_device_description");
        std::string port_name =
            get_serial_port_name(arduino_description, arduino_manufacturer);
        return "/dev/" + port_name;
    }
}

std::unique_ptr<ArduinoCommunication> initialize_triggering_with_default_params(
    const RecorderConfig &recorder_config,
    int muscle_num_lines_scanned,
    int sync_ratio,
    bool muscle_imaging_on) {
    spdlog::info("Starting Arduino communication");
    std::string arduino_port_name = find_arduino_port_name(recorder_config);
    std::unique_ptr<ArduinoCommunication> arduino_communication =
        std::make_unique<ArduinoCommunication>(arduino_port_name);
    spdlog::info("Arduino communication started");

    // Reboot the controller into a clean, known state before streaming. The
    // comm thread waits for the reboot and reopens the port, so the STREAM
    // below is delivered to the freshly reset controller.
    arduino_communication->reset();

    TriggerParams params = make_default_stream_params(
        recorder_config,
        muscle_num_lines_scanned,
        sync_ratio,
        muscle_imaging_on);
    spdlog::info(
        "Streaming default trigger params: enableMuscle={}, behFrameRate={}, "
        "behExpTime={} us, behMuscSyncRatio={}, muscEffExpTime={} us, "
        "pcoCamRollingTime={} us, pcoCamReadoutTime={} us",
        params.enable_muscle,
        params.beh_frame_rate,
        params.beh_exp_time,
        params.beh_musc_sync_ratio,
        params.musc_eff_exp_time,
        params.pco_cam_rolling_time,
        params.pco_cam_readout_time);
    arduino_communication->stream(params);
    spdlog::info("Arduino parameters set");

    return arduino_communication;
}