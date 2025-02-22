#include <string>
#include <tuple>
#include <sstream>
#include <vector>
#include <string>

enum CameraAcquisitionMode {
    STREAM,
    RECORD
};

const int behaviorCameraTriggerPin = 2;
const int irIlluminationTriggerPin = 3;
const int behaviorIndicatorPin = 6;
const int bufferFlushTimeMicrosecs = 100000;
int behaviorExposureTimeMicrosecs = 1000;
int behaviorCycleTimeMicrosecs = 1000000 / 25;
unsigned long lastBehaviorExposureStartTimeMicrosecs = 0;
bool behaviorTriggerState = LOW;
CameraAcquisitionMode acquisitionMode = STREAM;
bool waitingForCommand = false;

std::tuple<CameraAcquisitionMode, int, int> parseCommand(
    const std::string &commandString)
{
    std::vector<std::string> tokens;
    std::istringstream iss(commandString);
    for (std::string token; std::getline(iss, token, ' ');)
    {
        tokens.push_back(token);
    }
    if (tokens.size() != 3)
    {
        Serial.println("Arduino initialized successfully!");
        Serial.println("Invalid command: " + String(commandString.c_str()));
    }
    else
    {
        Serial.println("Received command: " + String(commandString.c_str()));
    }
    CameraAcquisitionMode mode = static_cast<CameraAcquisitionMode>(
        std::stoi(tokens[0]));
    int fps = std::stoi(tokens[1]);
    int exposureTimeMicrosecs = std::stoi(tokens[2]);

    return std::make_tuple(mode, fps, exposureTimeMicrosecs);
}

void setup() {
    Serial.begin(9600);

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
        String commandArduinoString = Serial.readStringUntil('\n');
        commandArduinoString.trim();
        std::string commandString(commandArduinoString.c_str());

        if (commandString == "PAUSE") {
            // Wait until another command is received
            waitingForCommand = true;
            Serial.println("PAUSE_ACK");
            return;
        } else {
            waitingForCommand = false;
        }

        auto [mode, fps, exposureTimeMicrosecs] = parseCommand(commandString);
        acquisitionMode = mode;
        behaviorCycleTimeMicrosecs = 1000000 / fps;
        behaviorExposureTimeMicrosecs = exposureTimeMicrosecs;

        if (mode == RECORD)
        {
            // Give it some time for remaining unprocessed frames to be
            // processed, so we know for sure that all newly arrived frames
            // are what we want.
            delayMicroseconds(bufferFlushTimeMicrosecs);
        }
    }
    
    // Blink logic
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

