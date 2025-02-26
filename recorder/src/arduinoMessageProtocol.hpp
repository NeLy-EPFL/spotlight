#ifndef ARDUINO_MESSAGE_INTERFACE_HPP
#define ARDUINO_MESSAGE_INTERFACE_HPP

#include <tuple>
#include <string>
#include <sstream>
#include <cstring>
#include <unordered_map>

enum ArduinoMessageType
{
    START_PULSING,
    STOP_PULSING,
    START_PULSING_ACK,
    STOP_PULSING_ACK,
    UNDEFINED,
};

// Note: The message type strings cannot be longer than 29 characters! Modify the size
// of messageTypeCString and change the string formatting pattern in the sscanf call
// in the ArduinoMessage constructor if you need to increase the size.
static const std::unordered_map<std::string, ArduinoMessageType>
    messageTypeStrToCode = {
        {"START_PULSING", START_PULSING},
        {"STOP_PULSING", STOP_PULSING},
        {"START_PULSING_ACK", START_PULSING_ACK},
        {"STOP_PULSING_ACK", STOP_PULSING_ACK},
        {"UNDEFINED", UNDEFINED}};

static const std::unordered_map<ArduinoMessageType, std::string>
    messageTypeCodeToStr = {
        {START_PULSING, "START_PULSING"},
        {STOP_PULSING, "STOP_PULSING"},
        {START_PULSING_ACK, "START_PULSING_ACK"},
        {STOP_PULSING_ACK, "STOP_PULSING_ACK"},
        {UNDEFINED, "UNDEFINED"}};

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