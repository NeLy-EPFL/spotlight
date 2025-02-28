#include <gtest/gtest.h>

#include "../src/arduinoMessageProtocol.hpp"

TEST(arduinoMessageProtocolTest, InitFromValues)
{
    ArduinoMessage message = ArduinoMessage(START_PULSING, 10, 2000);
    EXPECT_EQ(message.messageType, START_PULSING);
    EXPECT_EQ(message.pulseFrequency, 10);
    EXPECT_EQ(message.pulseWidth, 2000);
    EXPECT_EQ(message.isSyntaxValid, true);
}

TEST(arduinoMessageProtocolTest, InitFromCommandString)
{
    ArduinoMessage message = ArduinoMessage("STOP_PULSING -1 -1");
    EXPECT_EQ(message.messageType, STOP_PULSING);
    EXPECT_EQ(message.pulseFrequency, -1);
    EXPECT_EQ(message.pulseWidth, -1);
    EXPECT_EQ(message.isSyntaxValid, true);
}

TEST(arduinoMessageProtocolTest, ToCommandString)
{
    ArduinoMessage message = ArduinoMessage(START_PULSING_ACK);
    EXPECT_EQ(message.toCommString(), "START_PULSING_ACK -1 -1");
}

TEST(arduinoMessageProtocolTest, StopPulsingAck)
{
    ArduinoMessage message = ArduinoMessage("STOP_PULSING_ACK -1 -1");
    EXPECT_EQ(message.messageType, STOP_PULSING_ACK);
    EXPECT_EQ(message.isSyntaxValid, true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}