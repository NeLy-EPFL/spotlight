#include "arduinoCommunication.hpp"

namespace
{
    void arduinoCommThreadFunc(const std::string &portName,
                               int baudRate,
                               std::atomic<bool> &stopCommunication,
                               std::mutex &mutex,
                               std::condition_variable &cv,
                               std::queue<std::string> &arduinoMessagesQueue)
    {
        QSerialPort serialPort = QSerialPort();
        serialPort.setPortName(portName.c_str());
        serialPort.setBaudRate(baudRate);
        // serialPort.setDataBits(QSerialPort::Data8);
        // serialPort.setParity(QSerialPort::NoParity);
        // serialPort.setStopBits(QSerialPort::OneStop);
        // serialPort.setFlowControl(QSerialPort::NoFlowControl);

        spdlog::info("Opening Arduino comm on serial port {}", portName);
        if (!serialPort.open(QIODevice::ReadWrite))
        {
            spdlog::error("Failed to open serial port");
        }

        std::string incomingMessageBuffer;

        while (true)
        {
            std::string message;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [&]
                        { return !arduinoMessagesQueue.empty() ||
                                 stopCommunication; });
                if (stopCommunication)
                {
                    break;
                }
                message = arduinoMessagesQueue.front();
                arduinoMessagesQueue.pop();
            }
            serialPort.write(message.c_str());
            if (!serialPort.waitForBytesWritten(1000))
            {
                spdlog::error("Failed to write message to serial port");
            }

            // Check if there's any data to read from the Arduino
            if (serialPort.waitForReadyRead(1)) // Wait up to 1ms
            {
                QByteArray responseData = serialPort.readAll();
                if (!responseData.isEmpty())
                {
                    // Append new data to the buffer
                    incomingMessageBuffer += std::string(
                        responseData.constData(), responseData.length());

                    // Process complete lines
                    size_t newlinePos;
                    while ((newlinePos = incomingMessageBuffer.find('\n')) !=
                           std::string::npos)
                    {
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

        if (serialPort.isOpen())
        {
            serialPort.close();
        }
    }
}

ArduinoCommunication::ArduinoCommunication(const std::string &portName,
                                           int baudRate)
{
    arduinoCommThread_ = std::thread(arduinoCommThreadFunc,
                                     portName,
                                     baudRate,
                                     std::ref(stopCommunication_),
                                     std::ref(mutex_),
                                     std::ref(cv_),
                                     std::ref(arduinoMessagesQueue_));
}

ArduinoCommunication::~ArduinoCommunication()
{
    arduinoCommThread_.join();
}

void ArduinoCommunication::setBehaviorRecordingFPS(int fps)
{
    std::string message =
        ">SET_BEHAVIOR_FPS " + std::to_string(fps) + "\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::setSyncRatio(int syncRatio)
{
    std::string message =
        ">SET_SYNC_RATIO " + std::to_string(syncRatio) + "\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::setBehaviorExposureTime(int exposureTimeUs)
{
    std::string message =
        ">SET_BEHAVIOR_EXPOSURE_TIME " + std::to_string(exposureTimeUs) + "\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::setMuscleExposureTime(int exposureTimeUs)
{
    std::string message =
        ">SET_MUSCLE_EXPOSURE_TIME " + std::to_string(exposureTimeUs) + "\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::startRecording(
    std::vector<ProtocolStep> protocolSteps)
{
    std::string protocolString = generateProtocolString(protocolSteps);
    std::string message = ">START_RECORDING " + protocolString + "\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::stopRecording()
{
    std::string message = ">STOP_RECORDING\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        arduinoMessagesQueue_.push(message);
    }
    cv_.notify_one();
}

void ArduinoCommunication::stopCommunication()
{
    stopCommunication_ = true;
    cv_.notify_one();
}

std::string generateProtocolString(std::vector<ProtocolStep> protocolSteps)
{
    std::string protocolString;
    if (protocolSteps.empty())
    {
        protocolString = ";";
    }
    else
    {
        for (const auto &step : protocolSteps)
        {
            protocolString += step.toString() + ";";
        }
        // Remove last tangling semicolon
        protocolString.pop_back();
    }
    return protocolString;
}

std::string findArduinoPortName(RecorderConfig &recorderConfig)
{
    std::string arduinoManufacturer = recorderConfig.getParameter<std::string>(
        "triggering", "arduino_device_manufacturer");
    std::string arduinoDescription = recorderConfig.getParameter<std::string>(
        "triggering", "arduino_device_description");
    std::string portName = getSerialPortName(arduinoDescription,
                                             arduinoManufacturer);
    return "/dev/" + portName;
}