#include "recorder/peripherals/arduino_communication.h"

namespace {
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

    while (true) {
        std::string message;
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&] {
                return !arduinoMessagesQueue.empty() || stopCommunication;
            });
            if (stopCommunication) {
                break;
            }
            message = arduinoMessagesQueue.front();
            arduinoMessagesQueue.pop();
        }

        // Try sending the message if fails twice in a row reconnect to the
        // serial port
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

        // Check if there's any data to read from the Arduino
        if (serialPort.waitForReadyRead(1)) // Wait up to 1ms
        {
            QByteArray responseData = serialPort.readAll();
            if (!responseData.isEmpty()) {
                // Append new data to the buffer
                incomingMessageBuffer += std::string(
                    responseData.constData(), responseData.length());

                // Process complete lines
                size_t newlinePos;
                while ((newlinePos = incomingMessageBuffer.find('\n')) !=
                       std::string::npos) {
                    // Extract the complete line
                    std::string completeLine =
                        incomingMessageBuffer.substr(0, newlinePos);
                    // Remove the processed line from the buffer
                    incomingMessageBuffer.erase(0, newlinePos + 1);
                    // Log the complete line
                    spdlog::info("Arduino said: {}", completeLine);
                }
            }
        }
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
    arduinoCommThread_.join();
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

void ArduinoCommunication::stopExcitation() {
    TriggerParams params;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        params = lastStreamParams_;
    }
    params.muscEffExpTime = 0; // blue excitation LED never fires
    stream(params);
}

void ArduinoCommunication::stopCommunication() {
    stopCommunication_ = true;
    cv_.notify_one();
}

TriggerParams makeDefaultStreamParams(
    RecorderConfig &recorderConfig,
    int muscleNumLinesScanned,
    int syncRatio,
    bool muscleImagingOn) {
    TriggerParams params;
    params.behFrameRate = recorderConfig.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    params.behExpTime = recorderConfig.getParameter<int>(
        "behavior_camera", "default_exposure_time_us");
    params.behMuscSyncRatio = syncRatio >= 1 ? syncRatio : 1;
    int muscleLightOnTimeUs = recorderConfig.getParameter<int>(
        "muscle_camera", "default_light_on_time_us");
    params.muscEffExpTime = muscleImagingOn ? muscleLightOnTimeUs : 0;
    double rollingShutterLineTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "rolling_shutter_line_time_us");
    double sensorReadoutTimeUs = recorderConfig.getParameter<double>(
        "muscle_camera", "sensor_readout_time_us");
    params.pcoCamRollingTime =
        static_cast<unsigned int>(muscleNumLinesScanned * rollingShutterLineTimeUs);
    params.pcoCamReadoutTime = static_cast<unsigned int>(sensorReadoutTimeUs);
    return params;
}

std::string findArduinoPortName(RecorderConfig &recorderConfig) {
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
    RecorderConfig &recorderConfig, int muscleNumLinesScanned, int syncRatio) {
    spdlog::info("Starting Arduino communication");
    std::string arduinoPortName = findArduinoPortName(recorderConfig);
    std::unique_ptr<ArduinoCommunication> arduinoCommunication =
        std::make_unique<ArduinoCommunication>(arduinoPortName);
    spdlog::info("Arduino communication started");
    TriggerParams params = makeDefaultStreamParams(
        recorderConfig, muscleNumLinesScanned, syncRatio,
        /*muscleImagingOn=*/true);
    spdlog::info(
        "Streaming default trigger params: behFrameRate={}, behExpTime={} us, "
        "behMuscSyncRatio={}, muscEffExpTime={} us, pcoCamRollingTime={} us, "
        "pcoCamReadoutTime={} us",
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