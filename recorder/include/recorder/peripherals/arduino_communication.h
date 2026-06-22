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
    ArduinoCommunication(
        const std::string &port_name = "", int baud_rate = 9600);
    ~ArduinoCommunication();

    // Send a STREAM command (live preview). The params are cached so that
    // stop_excitation() can re-stream them with the excitation light off.
    void stream(const TriggerParams &params);

    // Send a START_RECORDING command. An empty op_sequence begins an open
    // recording (ended with stop_recording()); a non-empty one begins a
    // scheduled recording that reverts to revert_to_params on its STOP step.
    void start_recording(
        const TriggerParams &rec_params,
        const TriggerParams &revert_to_params,
        const std::deque<OperationStep> &op_sequence);

    // Send a STOP_RECORDING command. Per docs/comm_protocol.md this is only
    // valid while an open recording is in progress.
    void stop_recording();

    // Send a RESET command, rebooting the trigger controller (the firmware
    // calls esp_restart()) so it starts from a clean, known state. The recorder
    // issues this once at program startup. The reboot drops the controller's
    // USB CDC link, so the communication thread waits for it to come back and
    // reopens the serial port before sending any later command (e.g. the
    // initial STREAM).
    void reset();

    // Re-stream the most recently streamed params with the muscle camera
    // disabled (enableMuscle = false). This switches the blue excitation light
    // off and lets the behavior camera free-run on the controller's own clock,
    // while keeping the controller in a valid streaming state (the protocol has
    // no "stop triggering" command).
    void stop_excitation();

    void stop_communication();

  private:
    void enqueue_message(const std::string &message);

    QSerialPort *serial_port_;
    std::atomic<bool> stop_communication_ = false;
    std::thread arduino_comm_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::string> arduino_messages_queue_;
    TriggerParams last_stream_params_;
};

// Build a default streaming params struct from the recorder config. The PCO
// rolling time is derived from the muscle ROI height (number of scanned lines).
TriggerParams make_default_stream_params(
    const RecorderConfig &recorder_config,
    int muscle_num_lines_scanned,
    int sync_ratio,
    bool muscle_imaging_on);
std::string find_arduino_port_name(const RecorderConfig &recorder_config);
// muscle_imaging_on selects the controller's timing mode and defaults to off,
// to match the behavior-only streaming default of the firmware and the GUI:
// false gives free-running behavior-only acquisition; true gives muscle-synced
// acquisition where behavior frames are gated on the muscle camera's
// common-time signal. Only turn it on when a muscle camera is present,
// otherwise the behavior camera stalls waiting for a signal that never arrives.
std::unique_ptr<ArduinoCommunication> initialize_triggering_with_default_params(
    const RecorderConfig &recorder_config,
    int muscle_num_lines_scanned,
    int sync_ratio,
    bool muscle_imaging_on = false);
