#include "arduinoMessageProtocol.hpp"

ArduinoMessage::ArduinoMessage(std::string message)
{
    std::istringstream iss(message);
    std::string messageTypeStr;
    int pulseFrequencyLocal;
    int pulseWidthLocal;

    if (!(iss >> messageTypeStr >> pulseFrequencyLocal >> pulseWidthLocal) ||
        !(iss >> std::ws).eof())
    {
        isSyntaxValid = false;
        return;
    }

    auto it = messageTypeStrToCode.find(messageTypeStr);
    if (it != messageTypeStrToCode.end())
    {
        messageType = it->second;
        isSyntaxValid = true;
        if (messageType == START_PULSING)
        {
            pulseFrequency = pulseFrequencyLocal;
            pulseWidth = pulseWidthLocal;
        }
    }
    else
    {
        isSyntaxValid = false;
    }
}

ArduinoMessage::ArduinoMessage(
    ArduinoMessageType messageType,
    int pulseFrequency,
    int pulseWidth)
    : messageType(messageType),
      pulseFrequency(pulseFrequency),
      pulseWidth(pulseWidth)
{
    isSyntaxValid = true;
}

ArduinoMessage::ArduinoMessage(ArduinoMessageType messageType)
    : messageType(messageType)
{
    if (messageType == START_PULSING)
    {
        // START_PULSING must come with pauseFrequency and pulseWidth
        isSyntaxValid = false;
    }
    else
    {
        isSyntaxValid = true;
    }
}

std::string ArduinoMessage::toCommString()
{
    std::string messageTypeStr = messageTypeCodeToStr.at(messageType);

    std::ostringstream oss;
    oss << messageTypeStr << " " << pulseFrequency << " " << pulseWidth;
    return oss.str();
}