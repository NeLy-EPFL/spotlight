#include "triggering.hpp"

ArduinoTriggerControllerInterface::ArduinoTriggerControllerInterface()
{
    serialPortName_ = getSerialPortName(
        ARDUINO_DEVICE_DESCRIPTION, ARDUINO_DEVICE_MANUFACTURER);
    serialPort_.setPortName(QString::fromStdString(serialPortName_));
    serialPort_.setBaudRate(ARDUINO_BAUD_RATE_Q_ENUM);
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

void ArduinoTriggerControllerInterface::sendCommand(
    const std::string &command)
{
    if (serialPort_.isOpen())
    {
        serialPort_.write(QString::fromStdString(command).toUtf8() + '\n');
        spdlog::info("Message sent to Arduino: '{}'", command);
    }
    else
    {
        spdlog::critical("Failed to send command; serial port not open.");
        throw std::runtime_error(
            "Failed to send command; serial port not open.");
    }
}

ArduinoMessage ArduinoTriggerControllerInterface::waitForMessage(
    int timeOutMillisecs)
{
    for (int i = 0; i < ARDUINO_COMM_RETRIES; i++)
    {
        if (serialPort_.waitForReadyRead(timeOutMillisecs))
        {
            QByteArray response = serialPort_.readAll();
            std::string responseStr = response.trimmed().toStdString();
            spdlog::info("Message received from Arduino: '{}'", responseStr);
            if (!responseStr.empty())
            {
                ArduinoMessage responseMessage(responseStr);
                return responseMessage;
            }
            else
            {
                spdlog::warn(
                    "Received an empty message from Arduino. This could just "
                    "be harmless unflushed bits in the buffer; retrying...");
            }
        }
    }
    spdlog::critical(
        "Failed to receive message from Arduino within {} milliseconds. "
        "Retried {} times to no avail. Check communication with Arduino.",
        timeOutMillisecs, ARDUINO_COMM_RETRIES);
    throw std::runtime_error("Failed to receive message from Arduino.");
}

void ArduinoTriggerControllerInterface::startRecording(
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
    spdlog::info(
        "Arduino acknowledged the STOP_PAUSING command. I will now wait {} "
        "milliseconds to make sure the camera image acquisition thread is "
        "not receiving any frames anymore.",
        BUFFER_FLUSHING_WAIT_TIME_MILLISECS);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(BUFFER_FLUSHING_WAIT_TIME_MILLISECS));

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

    response = waitForMessage(1000);
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

void ArduinoTriggerControllerInterface::stopRecording(
    int recordingExposureTimeMicrosecs)
{
    isRecording.store(false);

    ArduinoMessage stopMessage(START_PULSING,
                               BEHAVIOR_CAMERA_STREAMING_FPS,
                               recordingExposureTimeMicrosecs);
    std::string commandString = stopMessage.toCommString();
    sendCommand(commandString);
    spdlog::info("Command sent to Arduino: {}", commandString);

    ArduinoMessage response = waitForMessage(1000);
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