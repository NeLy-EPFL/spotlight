#include "arduinoMessageInterface.hpp"

ArduinoMessage::ArduinoMessage(std::string message)
{
    char messageTypeCString[50];
    int pulseFrequencyLocal;
    int pulseWidthLocal;

    int parsed = sscanf(message.c_str(),
                        "%s %d %d",
                        messageTypeCString,
                        &pulseFrequencyLocal,
                        &pulseWidthLocal);
    if (parsed != 3)
    {
        isSyntaxValid = false;
        return;
    }

    if (strcmp(messageTypeCString, "START_PULSING") == 0)
    {
        messageType = START_PULSING;
        pulseFrequency = pulseFrequencyLocal;
        pulseWidth = pulseWidthLocal;
        isSyntaxValid = true;
    }
    else if (strcmp(messageTypeCString, "STOP_PULSING") == 0)
    {
        messageType = STOP_PULSING;
        isSyntaxValid = true;
    }
    else if (strcmp(messageTypeCString, "START_PULSING_ACK") == 0)
    {
        messageType = START_PULSING_ACK;
        isSyntaxValid = true;
    }
    else if (strcmp(messageTypeCString, "STOP_PULSING_ACK") == 0)
    {
        messageType = STOP_PULSING_ACK;
        isSyntaxValid = true;
    }
    else if (strcmp(messageTypeCString, "UNDEFINED") == 0)
    {
        messageType = UNDEFINED;
        isSyntaxValid = true;
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
    if (messageType == STOP_PULSING ||
        messageType == START_PULSING_ACK ||
        messageType == STOP_PULSING_ACK ||
        messageType == UNDEFINED)
    {
        isSyntaxValid = true;
    }
    else
    {
        // START_PULSING must come with pauseFrequency and pulseWidth
        isSyntaxValid = false;
    }
}

std::string ArduinoMessage::toCommString()
{
    std::string messageTypeStr;
    switch (messageType)
    {
    case START_PULSING:
        messageTypeStr = "START_PULSING";
        break;
    case STOP_PULSING:
        messageTypeStr = "STOP_PULSING";
        break;
    case START_PULSING_ACK:
        messageTypeStr = "START_PULSING_ACK";
        break;
    case STOP_PULSING_ACK:
        messageTypeStr = "STOP_PULSING_ACK";
        break;
    case UNDEFINED:
        messageTypeStr = "UNDEFINED";
        break;
    default:
        messageTypeStr = "UNDEFINED";
        break;
    }

    std::ostringstream oss;
    oss << messageTypeStr << " " << pulseFrequency << " " << pulseWidth;
    return oss.str();
}