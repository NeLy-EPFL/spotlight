#ifndef ARDUINO_COMMUNICATION_HPP
#define ARDUINO_COMMUNICATION_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <spdlog/spdlog.h>

#include "../common/dataTypes.hpp"
#include "../common/recorderConfig.hpp"
#include "../common/utils.hpp"
#include "experimentProtocol.hpp"

class ArduinoCommunication {
  public:
    ArduinoCommunication(const std::string &portName = "", int baudRate = 9600);
    ~ArduinoCommunication();

    void setBehaviorRecordingFPS(int fps);
    void setSyncRatio(int syncRatio);
    void setBehaviorExposureTime(int exposureTimeUs);
    void setMuscleLightOnTime(int lightOnTimeUs);
    void setMuscleCamTriggerDelay(int delayUs);
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

std::string generateProtocolString(std::vector<ProtocolStep> protocolSteps);
std::string findArduinoPortName(RecorderConfig &recorderConfig);
std::unique_ptr<ArduinoCommunication>
initializeTriggeringWithDefaultParams(RecorderConfig &recorderConfig, int muscleNumLinesScanned,
                                      int syncRatio);

#endif // ARDUINO_COMMUNICATION_HPP