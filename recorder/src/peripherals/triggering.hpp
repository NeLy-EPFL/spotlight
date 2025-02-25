#ifndef TRIGGERING_HPP
#define TRIGGERING_HPP

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <spdlog/spdlog.h>

#include "../constants.hpp"
#include "../global.hpp"
#include "../utils.hpp"
#include "../arduinoMessageInterface.hpp"

// Low-level functions to communicate with Arduino
std::string getSerialPortName(std::string deviceDescription,
                              std::string deviceManufacturer);
void sendCommand(QSerialPort &serialPort, const std::string &command);
ArduinoMessage waitForMessage(
    QSerialPort &serialPort, int timeOutMillisecs = ARDUINO_COMM_TIMEOUT_MILLISECS);

// High-level functions to run procedures that pause/restart triggering
// pulses at the right frquencies when recording starts or stops
void runRecordingStartProcedure(QSerialPort &serialPort,
                                int recordingFPS,
                                int recordingExposureTimeMicrosecs);
void runRecordingStopProcedure(QSerialPort &serialPort,
                               int recordingExposureTimeMicrosecs);

#endif // TRIGGERING_HPP