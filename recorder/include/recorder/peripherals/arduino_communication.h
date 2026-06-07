#pragma once

#include <condition_variable>
#include <deque>
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

#include <comm_protocol/protocol.h>

#include "recorder/common/data_types.h"
#include "recorder/common/recorder_config.h"
#include "recorder/common/utils.h"

class ArduinoCommunication {
  public:
    ArduinoCommunication(const std::string &portName = "", int baudRate = 9600);
    ~ArduinoCommunication();

    // Send a STREAM command (live preview). The params are cached so that
    // stopExcitation() can re-stream them with the excitation light off.
    void stream(const TriggerParams &params);

    // Send a START_RECORDING command. An empty opSequence begins an open
    // recording (ended with stopRecording()); a non-empty one begins a
    // scheduled recording that reverts to revertToParams on its STOP step.
    void startRecording(
        const TriggerParams &recParams,
        const TriggerParams &revertToParams,
        const std::deque<OperationStep> &opSequence);

    // Send a STOP_RECORDING command. Per docs/comm_protocol.md this is only
    // valid while an open recording is in progress.
    void stopRecording();

    // Send a RESET command, rebooting the trigger controller (the firmware calls
    // esp_restart()) so it starts from a clean, known state. The recorder issues
    // this once at program startup. The reboot drops the controller's USB CDC
    // link, so the communication thread waits for it to come back and reopens the
    // serial port before sending any later command (e.g. the initial STREAM).
    void reset();

    // Re-stream the most recently streamed params with the muscle camera
    // disabled (enableMuscle = false). This switches the blue excitation light
    // off and lets the behavior camera free-run on the controller's own clock,
    // while keeping the controller in a valid streaming state (the protocol has
    // no "stop triggering" command).
    void stopExcitation();

    void stopCommunication();

  private:
    void enqueueMessage(const std::string &message);

    QSerialPort *serialPort_;
    std::atomic<bool> stopCommunication_ = false;
    std::thread arduinoCommThread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::string> arduinoMessagesQueue_;
    TriggerParams lastStreamParams_;
};

// Build a default streaming params struct from the recorder config. The PCO
// rolling time is derived from the muscle ROI height (number of scanned lines).
TriggerParams makeDefaultStreamParams(
    RecorderConfig &recorderConfig,
    int muscleNumLinesScanned,
    int syncRatio,
    bool muscleImagingOn);
std::string findArduinoPortName(RecorderConfig &recorderConfig);
std::unique_ptr<ArduinoCommunication> initializeTriggeringWithDefaultParams(
    RecorderConfig &recorderConfig, int muscleNumLinesScanned, int syncRatio);
