#include <string>
#include <tuple>
#include <sstream>
#include <vector>
#include <string>

#include "arduinoMessageInterface.hpp"
#include "constants.hpp"

const int behaviorCameraTriggerPin = 2;
const int irIlluminationTriggerPin = 3;
const int behaviorIndicatorPin = 6;
int behaviorCycleTimeMicrosecs =
    1000000 / BEHAVIOR_CAMERA_STREAMING_FPS;
int behaviorExposureTimeMicrosecs =
    BEHAVIOR_CAMERA_DEFAULT_EXPOSURE_TIME_MICROSECS;
unsigned long lastBehaviorExposureStartTimeMicrosecs = 0;
bool behaviorTriggerState = LOW;
bool waitingForCommand = false;

void setup() {
    Serial.begin(ARDUINO_BAUD_RATE_INT);

    pinMode(behaviorCameraTriggerPin, OUTPUT);
    pinMode(irIlluminationTriggerPin, OUTPUT);
    pinMode(behaviorIndicatorPin, OUTPUT);
    digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
    digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
    digitalWrite(behaviorIndicatorPin, behaviorTriggerState);
}

void loop() {
    // Check for serial input (this only takes 1-2 us)
    if (Serial.available() || waitingForCommand) {
        while (true) {
            String commandArduinoString = Serial.readStringUntil('\n');
            commandArduinoString.trim();
            std::string commandString(commandArduinoString.c_str());
            // Serial.println("READBACK: " + String(commandString.c_str()));
            ArduinoMessage message(commandString.c_str());

            if (message.messageType == START_PULSING) {
                // Start pulsing at the specified frequency and width now!
                behaviorCycleTimeMicrosecs = 1000000 / message.pulseFrequency;
                behaviorExposureTimeMicrosecs = message.pulseWidth;
                waitingForCommand = false; // carry on
                ArduinoMessage response(START_PULSING_ACK);
                Serial.println(response.toCommString().c_str());
                return;
            }
            else if (message.messageType == STOP_PULSING) {
                behaviorTriggerState = LOW;
                digitalWrite(behaviorCameraTriggerPin, behaviorTriggerState);
                digitalWrite(irIlluminationTriggerPin, behaviorTriggerState);
                digitalWrite(behaviorIndicatorPin, behaviorTriggerState);
                waitingForCommand = true; // keep checking until told to restart
                ArduinoMessage response(STOP_PULSING_ACK);
                Serial.println(response.toCommString().c_str());
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
            digitalWrite(behaviorIndicatorPin, behaviorTriggerState);
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
            digitalWrite(behaviorIndicatorPin, behaviorTriggerState);
            lastBehaviorExposureStartTimeMicrosecs = currentTimeMicrosecs;
        }
    }
}

