#include "triggering.hpp"

namespace
{
    int getBaudRateQEnum(int baudRateInt)
    {
        switch (baudRateInt)
        {
        case 9600:
            return QSerialPort::Baud9600;
        case 19200:
            return QSerialPort::Baud19200;
        case 38400:
            return QSerialPort::Baud38400;
        case 57600:
            return QSerialPort::Baud57600;
        case 115200:
            return QSerialPort::Baud115200;
        default:
            spdlog::critical("Invalid baud rate: {}", baudRateInt);
            throw std::runtime_error("Invalid baud rate.");
        }
    }
}

ArduinoTriggerInterface::ArduinoTriggerInterface(
    const RecorderConfig &recorderConfig)
{
    recorderConfig_ = recorderConfig;
    std::string arduinoDeviceManufacturer =
        recorderConfig.getParameter<std::string>("triggering",
                                                 "arduino_device_manufacturer");
    std::string arduinoDeviceDescription =
        recorderConfig.getParameter<std::string>("triggering",
                                                 "arduino_device_description");
    int arduinoBaudRateInt =
        recorderConfig.getParameter<int>("triggering", "arduino_baud_rate");

    serialPortName_ = getSerialPortName(arduinoDeviceDescription,
                                        arduinoDeviceManufacturer);
    serialPort_.setPortName(QString::fromStdString(serialPortName_));
    serialPort_.setBaudRate(getBaudRateQEnum(arduinoBaudRateInt));
    serialPort_.setDataBits(QSerialPort::Data8);
    serialPort_.setParity(QSerialPort::NoParity);
    serialPort_.setStopBits(QSerialPort::OneStop);
    serialPort_.setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort_.open(QIODevice::ReadWrite))
    {
        spdlog::info("Serial port opened successfully.");
    }
    else
    {
        spdlog::critical("Failed to open serial port.");
        throw std::runtime_error("Failed to open serial port.");
    }
}

void ArduinoTriggerInterface::sendCommand(
    const std::string &command)
{
    if (serialPort_.isOpen())
    {
        serialPort_.write(QString::fromStdString(command).toUtf8() + '\n');
        serialPort_.flush();
        spdlog::info("Message sent to Arduino: '{}'", command);
    }
    else
    {
        spdlog::critical("Failed to send command; serial port not open.");
        throw std::runtime_error(
            "Failed to send command; serial port not open.");
    }
}

ArduinoMessage ArduinoTriggerInterface::waitForMessage()
{
    int timeoutMillisecs = recorderConfig_.getParameter<int>(
        "triggering", "arduino_comm_timeout_ms");
    int numRetries = recorderConfig_.getParameter<int>(
        "triggering", "arduino_comm_retries");

    for (int i = 0; i < numRetries; i++)
    {
        if (serialPort_.waitForReadyRead(timeoutMillisecs))
        {
            QByteArray response = serialPort_.readAll();
            QList<QByteArray> lines = response.split('\n');

            for (const QByteArray &line : lines)
            {
                std::string responseStr = line.trimmed().toStdString();
                if (!responseStr.empty())
                {
                    spdlog::info("Message received from Arduino: '{}'",
                                 responseStr);
                    ArduinoMessage responseMessage(responseStr);
                    if (responseMessage.isSyntaxValid)
                    {
                        return responseMessage;
                    }
                }
            }

            // If no valid message found in any line
            spdlog::warn(
                "No valid message found in Arduino response; retrying...");
        }
    }
    spdlog::critical(
        "Failed to receive message from Arduino within {} milliseconds. "
        "Retried {} times to no avail. Check communication with Arduino.",
        timeoutMillisecs, numRetries);
    throw std::runtime_error("Failed to receive message from Arduino.");
}

void ArduinoTriggerInterface::startRecording(
    int recordingFPS, int recordingExposureTimeMicrosecs)
{
    spdlog::info(
        "Preparing for recording. I'm telling Arduino to pause trigger "
        "pulses. I will wait until the camera image acquisition thread is "
        "not receiving any frames anymore; then I will tell Arduino to "
        "continue.");
    std::string commandString = ArduinoMessage(STOP_PULSING).toCommString();
    sendCommand(commandString);

    ArduinoMessage response = waitForMessage();
    if (response.messageType != STOP_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::critical(
            "Arduino didn't acknowledge the STOP_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to pause trigger pulses.");
    }
    int bufferFlushingWaitTimeMs = recorderConfig_.getParameter<int>(
        "triggering", "image_buffer_flushing_wait_time_ms");
    spdlog::info(
        "Arduino acknowledged the STOP_PAUSING command. I will now wait {} "
        "milliseconds to make sure the camera image acquisition thread is "
        "not receiving any frames anymore.",
        bufferFlushingWaitTimeMs);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(bufferFlushingWaitTimeMs));

    spdlog::info(
        "OK. I assume all pending frames have arrived. I'm marking my state "
        "as recording; this way, new frames that arrive now will be saved. "
        "I'm also telling Arduino to start sending trigger pulses again.");
    isRecording.store(true);
    ArduinoMessage startMessage(START_PULSING,
                                recordingFPS,
                                recordingExposureTimeMicrosecs);
    commandString = startMessage.toCommString();
    sendCommand(commandString);

    response = waitForMessage();
    if (response.messageType != START_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::critical(
            "Arduino didn't acknowledge the START_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to start trigger pulses.");
    }
    spdlog::info("Arduino acknowledged the START_PAUSING command.");
}

void ArduinoTriggerInterface::stopRecording(
    int recordingExposureTimeMicrosecs)
{
    isRecording.store(false);

    int streamingFrameRate = recorderConfig_.getParameter<int>(
        "behavior_camera", "streaming_frame_rate");
    ArduinoMessage stopMessage(START_PULSING,
                               streamingFrameRate,
                               recordingExposureTimeMicrosecs);
    std::string commandString = stopMessage.toCommString();
    sendCommand(commandString);
    spdlog::info("Command sent to Arduino: {}", commandString);

    ArduinoMessage response = waitForMessage();
    if (response.messageType != START_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::critical(
            "Arduino didn't acknowledge the START_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to start trigger pulses.");
    }
    spdlog::info("Arduino acknowledged the START_PAUSING command.");
}