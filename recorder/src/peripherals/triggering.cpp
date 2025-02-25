#include "triggering.hpp"

void sendCommand(QSerialPort &serialPort, const std::string &command)
{
    if (serialPort.isOpen())
    {
        serialPort.write(QString::fromStdString(command).toUtf8() + '\n');
        spdlog::info("Message sent to Arduino: '{}'", command);
    }
    else
    {
        spdlog::error("Failed to send command; serial port not open.");
    }
}

ArduinoMessage waitForMessage(QSerialPort &serialPort, int timeOutMillisecs)
{
    for (int i = 0; i < ARDUINO_COMM_RETRIES; i++)
    {
        if (serialPort.waitForReadyRead(timeOutMillisecs))
        {
            QByteArray response = serialPort.readAll();
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
    spdlog::error(
        "Failed to receive message from Arduino within {} milliseconds. "
        "Retried {} times to no avail. Check communication with Arduino.",
        timeOutMillisecs, ARDUINO_COMM_RETRIES);
    throw std::runtime_error("Failed to receive message from Arduino.");
}

void runRecordingStartProcedure(QSerialPort &serialPort,
                                int recordingFPS,
                                int recordingExposureTimeMicrosecs)
{
    spdlog::info(
        "Preparing for recording. I'm telling Arduino to pause trigger "
        "pulses. I will wait until the camera image acquisition thread is "
        "not receiving any frames anymore; then I will tell Arduino to "
        "continue.");
    std::string commandString = ArduinoMessage(STOP_PULSING).toCommString();
    sendCommand(serialPort, commandString);

    ArduinoMessage response = waitForMessage(serialPort);
    if (response.messageType != STOP_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::error(
            "Arduino didn't acknowledge the STOP_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to pause trigger pulses.");
        return;
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
    isRecording->store(true);
    ArduinoMessage startMessage(START_PULSING,
                                recordingFPS,
                                recordingExposureTimeMicrosecs);
    commandString = startMessage.toCommString();
    sendCommand(serialPort, commandString);

    response = waitForMessage(serialPort, 1000);
    if (response.messageType != START_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::error(
            "Arduino didn't acknowledge the START_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to start trigger pulses.");
        return;
    }
    spdlog::info("Arduino acknowledged the START_PAUSING command.");
}

void runRecordingStopProcedure(QSerialPort &serialPort,
                               int recordingExposureTimeMicrosecs)
{
    isRecording->store(false);

    ArduinoMessage stopMessage(START_PULSING,
                               BEHAVIOR_CAMERA_STREAMING_FPS,
                               recordingExposureTimeMicrosecs);
    std::string commandString = stopMessage.toCommString();
    sendCommand(serialPort, commandString);
    spdlog::info("Command sent to Arduino: {}", commandString);

    ArduinoMessage response = waitForMessage(serialPort, 1000);
    if (response.messageType != START_PULSING_ACK || !response.isSyntaxValid)
    {
        spdlog::error(
            "Arduino didn't acknowledge the START_PAUSING command. "
            "It responded with message '{}'.",
            response.toCommString());
        throw std::runtime_error("Failed to start trigger pulses.");
        return;
    }
    spdlog::info("Arduino acknowledged the START_PAUSING command.");
}