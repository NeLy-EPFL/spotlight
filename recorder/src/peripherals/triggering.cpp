#include "triggering.hpp"

std::string getSerialPortName(std::string deviceDescription,
                              std::string deviceManufacturer)
{
    std::vector<SerialPortInfo> allSerialPortInfo;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
    {
        std::string portName = port.portName().toStdString();
        std::string description = port.description().toStdString();
        std::string manufacturer = port.manufacturer().toStdString();
        if (description == deviceDescription &&
            manufacturer == deviceManufacturer)
        {
            spdlog::info(
                "Serial port found. "
                "Port name: '{}', description: '{}', manufacturer: '{}'",
                portName, description, manufacturer);
            return portName;
        }
        allSerialPortInfo.push_back({portName, description, manufacturer});
    }

    spdlog::error(
        "Arduino serial port not found. "
        "I'm looking for manufacturer '{}', description '{}'. "
        "Available ports are:",
        deviceManufacturer, deviceDescription);
    for (SerialPortInfo serialPortInfo : allSerialPortInfo)
    {
        spdlog::error(
            "* Port name: '{}', description: '{}', manufacturer: '{}'",
            serialPortInfo.portName,
            serialPortInfo.description,
            serialPortInfo.manufacturer);
    }
    return "";
}

void sendCommand(QSerialPort &serialPort, const QString &command)
{
    if (serialPort.isOpen())
    {
        serialPort.write(command.toUtf8() + '\n');
    }
    else
    {
        spdlog::error("Failed to send command; serial port not open.");
    }
}

void waitUntilMessageReceived(QSerialPort &serialPort,
                              const std::string &message)
{
    while (true)
    {
        if (serialPort.waitForReadyRead(1000))
        {
            QByteArray response = serialPort.readAll();
            std::string responseStr = response.toStdString();
            if (responseStr.find(message) != std::string::npos)
            {
                break;
            }
        }
        else
        {
            spdlog::error(
                "Timeout while waiting for message: '{}' from Arduino",
                message);
            throw std::runtime_error("Timeout while waiting for message");
        }
    }
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
    sendCommand(serialPort, "PAUSE");

    waitUntilMessageReceived(serialPort, "PAUSE_ACK");
    spdlog::info(
        "Arduino told me it has stopped sending pulses. I will now wait {} "
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
    CameraAcquisitionConfig cameraAcquisitionConfig(
        CameraAcquisitionMode::RECORD,
        recordingFPS,
        recordingExposureTimeMicrosecs);
    std::string commandString = cameraAcquisitionConfig.toCommandString();
    sendCommand(serialPort, QString(commandString.c_str()));
    spdlog::info("Command sent to Arduino: {}", commandString);
}

void runRecordingStopProcedure(QSerialPort &serialPort,
                               int recordingExposureTimeMicrosecs)
{
    isRecording->store(false);

    CameraAcquisitionConfig cameraAcquisitionConfig(
        CameraAcquisitionMode::STREAM,
        BEHAVIOR_CAMERA_STREAMING_FPS,
        recordingExposureTimeMicrosecs);
    std::string commandString = cameraAcquisitionConfig.toCommandString();
    sendCommand(serialPort, QString(commandString.c_str()));
    spdlog::info("Command sent to Arduino: {}", commandString);
}