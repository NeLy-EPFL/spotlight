#ifndef ARDUINO_COMMUNICATION_HPP
#define ARDUINO_COMMUNICATION_HPP

#include <string>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <spdlog/spdlog.h>

#include "experimentProtocol.hpp"
#include "../recorderConfig.hpp"
#include "../dataTypes.hpp"
#include "../utils.hpp"

class ArduinoCommunication
{
public:
    ArduinoCommunication(const std::string &portName = "",
                         int baudRate = 9600);
    ~ArduinoCommunication();

    void setBehaviorRecordingFPS(int fps);
    void setSyncRatio(int syncRatio);
    void setBehaviorExposureTime(int exposureTimeUs);
    void setMuscleExposureTime(int exposureTimeUs);
    void startRecording(std::vector<ProtocolStep> protocolSteps);
    void stopRecording();
    void stopCommunication();

private:
    QSerialPort *serialPort_;
    std::atomic<bool> stopCommunication_ = false;
    std::thread arduinoCommThread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::string> arduinoMessagesQueue_;
};

std::string findArduinoPortName(RecorderConfig &recorderConfig);

#endif // ARDUINO_COMMUNICATION_HPP