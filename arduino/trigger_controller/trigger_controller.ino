#include <string>
#include <tuple>
#include <sstream>
#include <vector>
#include <string>

#include "constants.h"
#include "arduinoMessageProtocol.hpp"

const int behaviorCameraTriggerPin = 2;
const int irIlluminationTriggerPin = 3;
const int optoTriggerPin = 6;

const int indicatorLEDPinRed = 10;
const int indicatorLEDPinGreen = 11;
const int indicatorLEDPinBlue = 12;

int behaviorCycleTimeMicrosecs =
    1000000 / BEHAVIOR_CAMERA_STREAMING_FRAME_RATE;
int behaviorExposureTimeMicrosecs =
    BEHAVIOR_CAMERA_DEFAULT_EXPOSURE_TIME_US;
unsigned long lastBehaviorExposureStartTimeMicrosecs = 0;
bool behaviorTriggerState = LOW;
bool waitingForCommand = false;

// unsigned long optoSequenceStartTime = 0;
// bool isRunningOptoSequence = false;
// const int optoToggleOnTime = 5 * 1000000;
// const int optoToggleOffTime = 10 * 1000000;
// const int allDoneToggleTime = 35 * 1000000;

unsigned long optoSequenceStartTime = 0;
bool isRunningOptoSequence = false;
const int optoInitialOffDuration = 5 * 1000000;
const int optoOnDuration = 5 * 1000000;
const int optoOffDuration = 25 * 1000000;
const int optoRepeatTimes = 10;

enum Color {
    RED,
    GREEN,
    BLUE,
    OFF
};

void setup() {
    Serial.begin(TRIGGERING_ARDUINO_BAUD_RATE);

    pinMode(behaviorCameraTriggerPin, OUTPUT);
    pinMode(irIlluminationTriggerPin, OUTPUT);
    pinMode(optoTriggerPin, OUTPUT);

    pinMode(indicatorLEDPinRed, OUTPUT);
    pinMode(indicatorLEDPinGreen, OUTPUT);
    pinMode(indicatorLEDPinBlue, OUTPUT);

    digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
    digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
    digitalWrite(optoTriggerPin, LOW);

    setLEDColor(GREEN);
}

void setLEDColor(Color color) {
    switch (color) {
    case RED:
        digitalWrite(indicatorLEDPinRed, HIGH);
        digitalWrite(indicatorLEDPinGreen, LOW);
        digitalWrite(indicatorLEDPinBlue, LOW);
        break;
    case GREEN:
        digitalWrite(indicatorLEDPinRed, LOW);
        digitalWrite(indicatorLEDPinGreen, HIGH);
        digitalWrite(indicatorLEDPinBlue, LOW);
        break;
    case BLUE:
        digitalWrite(indicatorLEDPinRed, LOW);
        digitalWrite(indicatorLEDPinGreen, LOW);
        digitalWrite(indicatorLEDPinBlue, HIGH);
        break;
    case OFF:
        digitalWrite(indicatorLEDPinRed, LOW);
        digitalWrite(indicatorLEDPinGreen, LOW);
        digitalWrite(indicatorLEDPinBlue, LOW);
        break;
    };
}

void loop() {
    // Check for serial input (this only takes 1-2 us)
    if (Serial.available() || waitingForCommand) {
        while (true) {
            String commandArduinoString = Serial.readStringUntil('\n');
            commandArduinoString.trim();
            std::string commandString(commandArduinoString.c_str()); 
            // // The following line is for debugging only. Using it with the
            // // actuall GUI will not work because now the Arduino is saying
            // // random things back to the computer that it does not expect.
            // // Instead, debug by using the Serial Monitor of the Arduino
            // // IDE to write messages to the board manually.
            // Serial.println("READBACK: " + String(commandString.c_str()));
            ArduinoMessage message(commandString.c_str());

            if (message.messageType == START_PULSING) {
                // Start pulsing at the specified frequency and width now!
                behaviorCycleTimeMicrosecs = 1000000 / message.pulseFrequency;
                behaviorExposureTimeMicrosecs = message.pulseWidth;
                waitingForCommand = false; // carry on
                setLEDColor(GREEN);

                if (!isRunningOptoSequence && message.pulseFrequency == 100)
                {
                    isRunningOptoSequence = true;
                    optoSequenceStartTime = micros();
                    digitalWrite(optoTriggerPin, LOW);
                    setLEDColor(BLUE);
                }

                ArduinoMessage response(START_PULSING_ACK);
                Serial.println(response.toCommString().c_str());
                Serial.flush();
                return;
            }
            else if (message.messageType == STOP_PULSING) {
                behaviorTriggerState = LOW;
                digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
                digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
                waitingForCommand = true; // keep checking until told to restart
                ArduinoMessage response(STOP_PULSING_ACK);
                Serial.println(response.toCommString().c_str());
                Serial.flush();
                return;
            }
        }
    }
    
    // Trigger logic
    unsigned long currentTimeMicrosecs = micros();
    if (currentTimeMicrosecs - lastBehaviorExposureStartTimeMicrosecs
            >= behaviorExposureTimeMicrosecs)
    {
        if (behaviorTriggerState == HIGH)
        {
            behaviorTriggerState = LOW;
            digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
            digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
        }
    }

    if (currentTimeMicrosecs - lastBehaviorExposureStartTimeMicrosecs
            >= behaviorCycleTimeMicrosecs)
    {
        if (behaviorTriggerState == LOW)
        {
            behaviorTriggerState = HIGH;
            digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
            digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
            lastBehaviorExposureStartTimeMicrosecs = currentTimeMicrosecs;
        }
    }

    if (isRunningOptoSequence) {
        if (currentTimeMicrosecs - optoSequenceStartTime < optoInitialOffDuration) {
            digitalWrite(optoTriggerPin, LOW);
        }
        else {
            for (int i = optoRepeatTimes; i >= 0; --i) {
                unsigned long startTimeThisCycle =
                    optoSequenceStartTime +
                    optoInitialOffDuration +
                    i * (optoOnDuration + optoOffDuration);

                if (currentTimeMicrosecs < startTimeThisCycle) {
                    continue;
                }

                if (i == optoRepeatTimes) {
                    isRunningOptoSequence = false;
                    behaviorCycleTimeMicrosecs = INT_MAX;
                    setLEDColor(OFF);
                    break;
                }

                if (currentTimeMicrosecs - startTimeThisCycle >= optoOnDuration) {
                    digitalWrite(optoTriggerPin, LOW);
                }
                else {
                    digitalWrite(optoTriggerPin, HIGH);
                }
            }
        }
    }
}
