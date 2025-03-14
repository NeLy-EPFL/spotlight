#ifndef ARDUINO_TRIGGER_CONTROLLER_INTERFACE_HPP
#define ARDUINO_TRIGGER_CONTROLLER_INTERFACE_HPP

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <spdlog/spdlog.h>
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>

#include "../recorderConfig.hpp"
#include "../global.hpp"
#include "../utils.hpp"
#include "../arduinoMessageProtocol.hpp"

/**
 * @brief Interface for controlling Arduino-based camera triggering
 *
 * This class provides an interface for controlling the triggering of a camera
 * via an Arduino. It encapsulates the communication with the Arduino and
 * provides high-level functions for recording procedures.
 */
class ArduinoTriggerControllerInterface
{
public:
    /**
     * @brief Construct a new Arduino Trigger Controller Interface
     */
    ArduinoTriggerControllerInterface(const RecorderConfig &recorderConfig);

    /**
     * @brief Start the recording procedure
     *
     * Pauses trigger pulses, waits for buffer flushing, then resumes pulses
     * at recording settings
     *
     * @param recordingFPS The frames per second for the recording
     * @param recordingExposureTimeMicrosecs Exposure time in microseconds
     * @throws std::runtime_error if communication with Arduino fails
     */
    void startRecording(int recordingFPS, int recordingExposureTimeMicrosecs);

    /**
     * @brief Stop the recording procedure
     *
     * Stops recording and returns to streaming mode
     *
     * @param recordingExposureTimeMicrosecs Exposure time in microseconds
     * @throws std::runtime_error if communication with Arduino fails
     */
    void stopRecording(int recordingExposureTimeMicrosecs);

private:
    std::string serialPortName_;
    QSerialPort serialPort_;
    RecorderConfig recorderConfig_;

    /**
     * @brief Send a command to the Arduino
     *
     * @param command The command to send
     */
    void sendCommand(const std::string &command);

    /**
     * @brief Wait for a message from the Arduino
     *
     * @return ArduinoMessage The received message
     * @throws std::runtime_error if no message is received
     */
    ArduinoMessage waitForMessage();
};

#endif // ARDUINO_TRIGGER_CONTROLLER_INTERFACE_HPP