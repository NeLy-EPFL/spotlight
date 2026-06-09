#include "recorder/peripherals/arduino_communication.h"

namespace {
// Read whatever bytes the controller has sent and log each complete (newline-
// terminated) line at info level. `buffer` carries any partial trailing line
// across calls, so a message split across reads is reassembled. Waits up to 1 ms
// for bytes to arrive, then returns; safe to call when nothing is pending.
void drainIncomingSerial(QSerialPort &serialPort, std::string &buffer) {
    if (!serialPort.waitForReadyRead(1)) {
        return;
    }
    QByteArray responseData = serialPort.readAll();
    if (responseData.isEmpty()) {
        return;
    }
    buffer += std::string(responseData.constData(), responseData.length());
    size_t newlinePos;
    while ((newlinePos = buffer.find('\n')) != std::string::npos) {
        std::string completeLine = buffer.substr(0, newlinePos);
        buffer.erase(0, newlinePos + 1);
        spdlog::info("Arduino said: {}", completeLine);
    }
}

// After a RESET command, the firmware calls esp_restart() and the controller
// reboots, which tears down and re-creates its native USB CDC serial port. Wait
// for it to come back, then reopen the (stable) port so that subsequent commands
// reach the rebooted controller. Reopening relies on a stable port name -- the
// /dev/arduino_trigger udev symlink -- surviving the re-enumeration.
void waitForControllerReboot(
    QSerialPort &serialPort,
    const std::string &portName,
    std::atomic<bool> &stopCommunication) {
    // Time for the MCU to reboot and its USB CDC port to re-enumerate before we
    // try to reopen it; generous so the first reopen attempt below succeeds.
    constexpr auto rebootWait = std::chrono::milliseconds(2000);
    constexpr auto reopenInterval = std::chrono::milliseconds(200);
    constexpr int maxReopenAttempts = 50; // ~10 s total before giving up

    spdlog::info(
        "Trigger controller resetting (esp_restart); waiting for it to reboot "
        "and re-enumerate on {}",
        portName);
    serialPort.close();
    std::this_thread::sleep_for(rebootWait);

    for (int attempt = 0; attempt < maxReopenAttempts; ++attempt) {
        if (stopCommunication) {
            return;
        }
        if (serialPort.open(QIODevice::ReadWrite)) {
            spdlog::info("Reconnected to trigger controller after reset");
            return;
        }
        std::this_thread::sleep_for(reopenInterval);
    }
    spdlog::error(
        "Trigger controller did not re-enumerate within timeout after reset; "
        "will keep retrying on the next command write");
}

void arduinoCommThreadFunc(
    const std::string &portName,
    int baudRate,
    std::atomic<bool> &stopCommunication,
    std::mutex &mutex,
    std::condition_variable &cv,
    std::queue<std::string> &arduinoMessagesQueue) {
    QSerialPort serialPort = QSerialPort();
    serialPort.setPortName(portName.c_str());
    serialPort.setBaudRate(baudRate);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    spdlog::info("Opening Arduino comm on serial port {}", portName);
    if (!serialPort.open(QIODevice::ReadWrite)) {
        spdlog::error("Failed to open serial port");
    }

    // Reset Arduino
    serialPort.setDataTerminalReady(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serialPort.setDataTerminalReady(true);

    std::string incomingMessageBuffer;

    // The controller frames commands by newline; reset() enqueues the RESET
    // command the same way, so it can be recognized here by its exact serialized
    // form to drive the post-reboot reconnect (see waitForControllerReboot).
    const std::string resetCommandLine =
        Command::makeResetCommand().toString() + "\n";

    while (true) {
        std::string message;
        bool hasMessage = false;
        {
            std::unique_lock<std::mutex> lock(mutex);
            // Wake on a new outgoing command or on shutdown, but also time out
            // periodically so incoming bytes are drained even while idle (the
            // host sends nothing during steady-state streaming/recording). A
            // notify still returns immediately, so command sending stays prompt.
            cv.wait_for(lock, std::chrono::milliseconds(10), [&] {
                return !arduinoMessagesQueue.empty() || stopCommunication;
            });
            if (stopCommunication) {
                break;
            }
            if (!arduinoMessagesQueue.empty()) {
                message = arduinoMessagesQueue.front();
                arduinoMessagesQueue.pop();
                hasMessage = true;
            }
        }

        // Send the outgoing command, if any. On repeated write failures, try
        // reconnecting to the serial port.
        if (hasMessage) {
            int consecutiveFailedWrites = 0;
            int maxConsecutiveFailedWrites = 2;
            while (true) {
                serialPort.write(message.c_str());
                if (!serialPort.waitForBytesWritten(1000)) {
                    spdlog::error(
                        "Failed to write message: {} to serial port", message);
                    consecutiveFailedWrites++;
                    if (consecutiveFailedWrites >= maxConsecutiveFailedWrites) {
                        spdlog::error("Max consecutive failed writes reached. "
                                      "Reconnecting...");
                        serialPort.close();
                        if (!serialPort.open(QIODevice::ReadWrite)) {
                            spdlog::error(
                                "Failed to reopen serial port. Stopping Arduino "
                                "communication thread.");
                            break;
                        }
                        consecutiveFailedWrites = 0;
                        spdlog::info("Successfully reconnected to serial port.");
                    }
                    continue;
                }
                consecutiveFailedWrites = 0;
                break;
            }

            // A RESET reboots the controller and drops the serial link; wait for
            // it to come back and reopen the port before sending anything else.
            if (message == resetCommandLine) {
                waitForControllerReboot(
                    serialPort, portName, stopCommunication);
            }
        }

        // Drain incoming bytes every iteration -- after a send and on idle
        // wake-ups -- so the controller's USB-CDC TX buffer never backs up. A
        // full TX buffer can block Serial writes inside the firmware's timing
        // loop and stall triggering, so prompt draining protects timing.
        drainIncomingSerial(serialPort, incomingMessageBuffer);
    }

    if (serialPort.isOpen()) {
        serialPort.close();
    }
}
} // namespace

ArduinoCommunication::ArduinoCommunication(
    const std::string &portName, int baudRate) {
    arduinoCommThread_ = std::thread(
        arduinoCommThreadFunc,
        portName,
        baudRate,
        std::ref(stopCommunication_),
        std::ref(mutex_),
        std::ref(cv_),
        std::ref(arduinoMessagesQueue_));
}

ArduinoCommunication::~ArduinoCommunication() {
    // Always signal the comm thread to stop before joining it. Without this, an
    // owner that forgets to call stopCommunication() (as the run-spotlight
    // shutdown path does) would deadlock here forever, since the thread's loop
    // only exits once stopCommunication_ is set. stopCommunication() is
    // idempotent, so this is harmless when the owner already called it.
    stopCommunication();
    if (arduinoCommThread_.joinable()) {
        arduinoCommThread_.join();
    }
}

void ArduinoCommunication::enqueueMessage(const std::string &message) {
    // The controller frames commands by newline (see trigger_firmware
    // SerialIO), so every JSON command is terminated with one.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message + "\n");
    }
    cv_.notify_one();
}

void ArduinoCommunication::stream(const TriggerParams &params) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastStreamParams_ = params;
    }
    enqueueMessage(Command::makeStreamCommand(params).toString());
}

void ArduinoCommunication::startRecording(
    const TriggerParams &recParams,
    const TriggerParams &revertToParams,
    const std::deque<OperationStep> &opSequence) {
    enqueueMessage(
        Command::makeStartRecordingCommand(recParams, revertToParams, opSequence)
            .toString());
}

void ArduinoCommunication::stopRecording() {
    enqueueMessage(Command::makeStopRecordingCommand().toString());
}

void ArduinoCommunication::reset() {
    // Goes through the same FIFO queue as every other command, so any command
    // enqueued afterwards (e.g. the initial STREAM) is sent only once the comm
    // thread has waited out the reboot and reopened the port.
    enqueueMessage(Command::makeResetCommand().toString());
}

void ArduinoCommunication::stopExcitation() {
    TriggerParams params;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        params = lastStreamParams_;
    }
    // Disable the muscle camera entirely: the controller free-runs the behavior
    // camera on its own clock and never pulses the blue excitation LED.
    params.enableMuscle = false;
    stream(params);
}

void ArduinoCommunication::stopCommunication() {
    stopCommunication_ = true;
    cv_.notify_one();
}

TriggerParams makeDefaultStreamParams(
    const RecorderConfig &recorderConfig,
    int muscleNumLinesScanned,
    int syncRatio,
    bool muscleImagingOn) {
    TriggerParams params;
    params.behFrameRate = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    params.behExpTime = recorderConfig.getParameter<int>(
        "behavior_camera", "default_exposure_time_us");
    params.behMuscSyncRatio = syncRatio >= 1 ? syncRatio : 1;
    // enableMuscle selects the controller's mode; when false the muscle-only
    // fields below are sent but ignored (the behavior camera free-runs).
    params.enableMuscle = muscleImagingOn;
    int muscleLightOnTimeUs = recorderConfig.getParameter<int>(
        "muscle_camera", "default_light_on_time_us");
    params.muscEffExpTime = muscleLightOnTimeUs;
    double rollingShutterLineTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensorReadoutTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    params.pcoCamRollingTime =
        static_cast<unsigned int>(muscleNumLinesScanned * rollingShutterLineTimeUs);
    params.pcoCamReadoutTime = static_cast<unsigned int>(sensorReadoutTimeUs);
    return params;
}

std::string findArduinoPortName(const RecorderConfig &recorderConfig) {
    std::string udev_port_name = "/dev/arduino_trigger";
    if (fs::exists(udev_port_name)) {
        spdlog::info(
            "Serial port '{}' found via udev. Using it without further checks.",
            udev_port_name);
        return udev_port_name;
    } else {
        std::string arduinoManufacturer =
            recorderConfig.getParameter<std::string>(
                "triggering", "arduino_device_manufacturer");
        std::string arduinoDescription =
            recorderConfig.getParameter<std::string>(
                "triggering", "arduino_device_description");
        std::string portName =
            getSerialPortName(arduinoDescription, arduinoManufacturer);
        return "/dev/" + portName;
    }
}

std::unique_ptr<ArduinoCommunication> initializeTriggeringWithDefaultParams(
    const RecorderConfig &recorderConfig,
    int muscleNumLinesScanned,
    int syncRatio,
    bool muscleImagingOn) {
    spdlog::info("Starting Arduino communication");
    std::string arduinoPortName = findArduinoPortName(recorderConfig);
    std::unique_ptr<ArduinoCommunication> arduinoCommunication =
        std::make_unique<ArduinoCommunication>(arduinoPortName);
    spdlog::info("Arduino communication started");

    // Reboot the controller into a clean, known state before streaming. The
    // comm thread waits for the reboot and reopens the port, so the STREAM below
    // is delivered to the freshly reset controller.
    arduinoCommunication->reset();

    TriggerParams params = makeDefaultStreamParams(
        recorderConfig, muscleNumLinesScanned, syncRatio, muscleImagingOn);
    spdlog::info(
        "Streaming default trigger params: enableMuscle={}, behFrameRate={}, "
        "behExpTime={} us, behMuscSyncRatio={}, muscEffExpTime={} us, "
        "pcoCamRollingTime={} us, pcoCamReadoutTime={} us",
        params.enableMuscle,
        params.behFrameRate,
        params.behExpTime,
        params.behMuscSyncRatio,
        params.muscEffExpTime,
        params.pcoCamRollingTime,
        params.pcoCamReadoutTime);
    arduinoCommunication->stream(params);
    spdlog::info("Arduino parameters set");

    return arduinoCommunication;
}