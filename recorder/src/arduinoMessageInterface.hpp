#ifndef ARDUINO_MESSAGE_INTERFACE_HPP
#define ARDUINO_MESSAGE_INTERFACE_HPP

#include <tuple>
#include <string>
#include <sstream>
#include <cstring>

enum ArduinoMessageType
{
    START_PULSING,
    STOP_PULSING,
    START_PULSING_ACK,
    STOP_PULSING_ACK,
    UNDEFINED,
};

class ArduinoMessage
{
public:
    ArduinoMessage(std::string message);
    ArduinoMessage(
        ArduinoMessageType messageType, int pulseFrequency, int pulseWidth);
    ArduinoMessage(ArduinoMessageType messageType);
    std::string toCommString();

    ArduinoMessageType messageType = UNDEFINED;
    int pulseFrequency = -1;
    int pulseWidth = -1;
    bool isSyntaxValid = false;
};

#endif // ARDUINO_MESSAGE_INTERFACE_HPP