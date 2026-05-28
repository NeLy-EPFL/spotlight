#include <Arduino.h>

#include <string>
#include <vector>

#include "experimentProtocol.hpp"
#include "pinAssignment.hpp"
#include "util.hpp"

const int MAX_PROGRAM_STEPS = 1024;
const int INCOMING_MESSAGE_BUFFER_SIZE = 16384;

const int FRAME_BUFFER_FLUSH_TIME_US = 100000;

// Default parameters
// Period (1/FPS) of behavior camera, and period of muscle camera defined
// through syncRatioK (muscle period = behavior period * syncRatioK)
volatile unsigned int behaviorCamPeriod = 1000000 / 25; // in microseconds
volatile unsigned int syncRatioK = 1;

// Exposure times for cameras
volatile unsigned int behaviorCamExposureTime = 1000; // in microseconds
volatile unsigned int muscleCamExposureTime = 3000;   // in microseconds

// Muscle camera trigger should have a delay to make sure that the common time
// is aligned with the excitation-on period
volatile unsigned int muscleCamTriggerDelayUs = 0;

// Protocol steps
std::vector<ProtocolStep> protocolSteps;

// Timing variables
volatile unsigned long lastBehaviorTriggerTime = 0;
volatile unsigned long lastMuscleLightTriggerTime = 0;
volatile unsigned long lastMuscleCamTriggerTime = 0;
volatile unsigned long behaviorTriggerCounter = 0;
volatile unsigned long muscleCamTriggerDelayStartTime = 0;
volatile bool triggerMuscleCamThisCycle = false;
volatile bool behaviorTriggerState = false;
volatile bool muscleLightTriggerState = false;
volatile bool muscleCamTriggerState = false;

char incomingMessageBuffer[INCOMING_MESSAGE_BUFFER_SIZE];
int incomingMessageBufferIdx = 0;

// ---------------------------------------------------------------------------
// Low-level trigger helpers
// ---------------------------------------------------------------------------

void behaviorTriggerOn() {
    digitalWrite(BEHAVIOR_CAM_PIN, HIGH);
    digitalWrite(IR_LIGHT_PIN, HIGH);
}

void behaviorTriggerOff() {
    digitalWrite(BEHAVIOR_CAM_PIN, LOW);
    digitalWrite(IR_LIGHT_PIN, LOW);
}

void muscleLightTriggerOn() {
    digitalWrite(BLUE_LIGHT_PIN, HIGH);
}

void muscleLightTriggerOff() {
    digitalWrite(BLUE_LIGHT_PIN, LOW);
}

void muscleCamTriggerOn() {
    digitalWrite(MUSCLE_CAM_PIN, HIGH);
}

void muscleCamTriggerOff() {
    digitalWrite(MUSCLE_CAM_PIN, LOW);
}

// ---------------------------------------------------------------------------
// Command handlers (one per command)
// Each receives the argument portion of the message (everything after the
// command prefix and its trailing space).
// ---------------------------------------------------------------------------

void handleSetBehaviorExposureTime(const char *arg) {
    unsigned int expTime = atoi(arg);
    if (expTime > 0) {
        behaviorCamExposureTime = expTime;
        Serial.print("Behavior camera exposure time set to: ");
        Serial.println(behaviorCamExposureTime);
        Serial.flush();
        // litStatusLED(GREEN);
    } else {
        Serial.println(
            "Invalid exposure time for behavior camera: " + String(expTime));
        Serial.flush();
        litStatusLED(RED);
    }
}

void handleSetMuscleLightOnTime(const char *arg) {
    unsigned int lightOnTime = atoi(arg);
    if (lightOnTime > 0) {
        muscleCamExposureTime = lightOnTime;
        Serial.print("Muscle camera exposure time set to: ");
        Serial.println(muscleCamExposureTime);
        Serial.flush();
        // litStatusLED(GREEN);
    } else {
        Serial.print("Invalid exposure time for muscle camera: ");
        Serial.println(lightOnTime);
        Serial.flush();
        litStatusLED(RED);
    }
}

void handleSetBehaviorFps(const char *arg) {
    unsigned int fps = atoi(arg);
    if (fps == 0) {
        behaviorCamPeriod = UINT_MAX;
        litStatusLED(WHITE);
    } else {
        behaviorCamPeriod = 1000000 / fps;
    }
    Serial.print("Behavior camera FPS set to: ");
    Serial.println(fps);
    Serial.flush();
    litStatusLED(GREEN);
}

void handleSetSyncRatio(const char *arg) {
    unsigned int ratio = atoi(arg);
    if (ratio > 0) {
        syncRatioK = ratio;
        Serial.print("Sync ratio set to: ");
        Serial.println(syncRatioK);
        Serial.flush();
        litStatusLED(GREEN);
    } else {
        Serial.print("Invalid sync ratio: ");
        Serial.println(ratio);
        Serial.flush();
        litStatusLED(RED);
    }
}

void handleSetMuscleCamTriggerDelay(const char *arg) {
    unsigned int delayRequested = atoi(arg);
    muscleCamTriggerDelayUs = delayRequested;
    Serial.print("Muscle cam trigger delay set to: ");
    Serial.println(muscleCamTriggerDelayUs);
    Serial.flush();
    litStatusLED(GREEN);
}

void handleStartRecording(const char *arg) {
    int numStepsParsed = parseProtocolSequence(arg, protocolSteps);
    if (numStepsParsed == 0) {
        Serial.println("Received empty protocol; will record openly.");
        Serial.flush();
        litStatusLED(BLUE);
    } else if (numStepsParsed > 0) {
        Serial.print("Protocol sequence parsed successfully with ");
        Serial.print(numStepsParsed);
        Serial.println(" steps.");
        Serial.flush();
        litStatusLED(MAGENTA);
    } else {
        Serial.println("Failed to parse protocol sequence.");
        Serial.flush();
        litStatusLED(RED);
        return;
    }
    if (numStepsParsed > MAX_PROGRAM_STEPS) {
        Serial.print("Protocol sequence too long: ");
        Serial.print(numStepsParsed);
        Serial.print(" steps, max is ");
        Serial.println(MAX_PROGRAM_STEPS);
        Serial.flush();
        litStatusLED(RED);
        return;
    }

    behaviorTriggerCounter = 0;
    lastBehaviorTriggerTime = 0;
    lastMuscleLightTriggerTime = 0;
    lastMuscleCamTriggerTime = 0;
    behaviorTriggerState = false;
    muscleLightTriggerState = false;
    muscleCamTriggerState = false;
    Serial.println("Pausing to let frame buffer clear...");
    Serial.flush();
    behaviorTriggerOff();
    muscleLightTriggerOff();
    muscleCamTriggerOff();
    delayMicroseconds(FRAME_BUFFER_FLUSH_TIME_US);
    Serial.println("Recording started.");
    Serial.flush();
}

void handleStopRecording() {
    protocolSteps.clear();
    behaviorTriggerCounter = 0;
    Serial.println("Recording stopped.");
    Serial.flush();
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void parseIncomingCommand(const char *message) {
    if (strncmp(
            message,
            CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME,
            CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME) == 0)
        handleSetBehaviorExposureTime(
            message + CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME + 1);
    else if (
        strncmp(
            message,
            CMDSTR_SET_MUSCLE_LIGHT_ON_TIME,
            CMDLEN_SET_MUSCLE_LIGHT_ON_TIME) == 0)
        handleSetMuscleLightOnTime(
            message + CMDLEN_SET_MUSCLE_LIGHT_ON_TIME + 1);
    else if (
        strncmp(message, CMDSTR_SET_BEHAVIOR_FPS, CMDLEN_SET_BEHAVIOR_FPS) == 0)
        handleSetBehaviorFps(message + CMDLEN_SET_BEHAVIOR_FPS + 1);
    else if (
        strncmp(message, CMDSTR_SET_SYNC_RATIO, CMDLEN_SET_SYNC_RATIO) == 0)
        handleSetSyncRatio(message + CMDLEN_SET_SYNC_RATIO + 1);
    else if (
        strncmp(
            message,
            CMDSTR_SET_MUSCLE_CAM_TRIGGER_DELAY,
            CMDLEN_SET_MUSCLE_CAM_TRIGGER_DELAY) == 0)
        handleSetMuscleCamTriggerDelay(
            message + CMDLEN_SET_MUSCLE_CAM_TRIGGER_DELAY + 1);
    else if (
        strncmp(message, CMDSTR_START_RECORDING, CMDLEN_START_RECORDING) == 0)
        handleStartRecording(message + CMDLEN_START_RECORDING + 1);
    else if (
        strncmp(message, CMDSTR_STOP_RECORDING, CMDLEN_STOP_RECORDING) == 0)
        handleStopRecording();
    else {
        Serial.print("Arduino received the following command: '");
        Serial.print(message);
        Serial.println("' by serial comm.");
        Serial.flush();
        litStatusLED(RED);
    }
}

void filterAndProcessIncomingMessage(const char *message) {
    if (message[0] == '>') {
        parseIncomingCommand(message);
    } else {
        Serial.print("Arduino received the following message: '");
        Serial.print(message);
        Serial.println("' by serial comm.");
        Serial.flush();
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Set pin modes
    pinMode(BEHAVIOR_CAM_PIN, OUTPUT);
    pinMode(IR_LIGHT_PIN, OUTPUT);
    pinMode(MUSCLE_CAM_PIN, OUTPUT);
    pinMode(BLUE_LIGHT_PIN, OUTPUT);
    pinMode(OPTO_CH2_PIN, OUTPUT);
    pinMode(OPTO_CH3_PIN, OUTPUT);
    pinMode(GREEN_LIGHT_PIN, OUTPUT);
    pinMode(STATUS_LED_RED_PIN, OUTPUT);
    pinMode(STATUS_LED_GREEN_PIN, OUTPUT);
    pinMode(STATUS_LED_BLUE_PIN, OUTPUT);

    // Initialize pins to LOW
    behaviorTriggerOff();
    muscleLightTriggerOff();
    muscleCamTriggerOff();
    digitalWrite(OPTO_CH2_PIN, LOW);
    digitalWrite(OPTO_CH3_PIN, LOW);
    digitalWrite(GREEN_LIGHT_PIN, LOW);
    digitalWrite(STATUS_LED_RED_PIN, LOW);
    digitalWrite(STATUS_LED_GREEN_PIN, LOW);
    digitalWrite(STATUS_LED_BLUE_PIN, LOW);

    litStatusLED(GREEN);
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (updateCommandBuffer(
                c,
                incomingMessageBuffer,
                incomingMessageBufferIdx,
                INCOMING_MESSAGE_BUFFER_SIZE)) {
            filterAndProcessIncomingMessage(incomingMessageBuffer);
        }
    }

    unsigned long currentTime = micros();

    // Should I open behavior camera shutter?
    if (currentTime - lastBehaviorTriggerTime >= behaviorCamPeriod &&
        !behaviorTriggerState) {
        behaviorTriggerOn();
        behaviorTriggerState = true;
        lastBehaviorTriggerTime = currentTime;

        // Should I mark the muscle *CAMERA* as to be triggered during this
        // cycle?
        if (behaviorTriggerCounter % syncRatioK == 0) {
            triggerMuscleCamThisCycle = true;
            // Use (currentTime - delayStartTime >= delayTime) instead of
            // addition to ensure proper wrapping when counter overflows
            muscleCamTriggerDelayStartTime = currentTime;
        }

        // Should I trigger muscle *EXCITATION LIGHT* this cycle?
        if (behaviorTriggerCounter % syncRatioK == 0 &&
            !muscleLightTriggerState) {
            muscleLightTriggerOn();
            muscleLightTriggerState = true;
            lastMuscleLightTriggerTime = currentTime;
        }

        behaviorTriggerCounter++;
    }

    // Should I open muscle camera shutter?
    if (triggerMuscleCamThisCycle &&
        currentTime - muscleCamTriggerDelayStartTime >=
            muscleCamTriggerDelayUs &&
        !muscleCamTriggerState) {
        muscleCamTriggerOn();
        muscleCamTriggerState = true;
        lastMuscleCamTriggerTime = currentTime;
        triggerMuscleCamThisCycle = false; // Reset for the next cycle
    }

    // Should I close behavior camera shutter?
    if (currentTime - lastBehaviorTriggerTime >= behaviorCamExposureTime &&
        behaviorTriggerState) {
        behaviorTriggerOff();
        behaviorTriggerState = false;
    }

    // Should I turn off muscle excitation light?
    if (currentTime - lastMuscleLightTriggerTime >= muscleCamExposureTime &&
        muscleLightTriggerState) {
        muscleLightTriggerOff();
        muscleLightTriggerState = false;
    }

    // Should I close muscle camera shutter?
    if (currentTime - lastMuscleCamTriggerTime >= muscleCamExposureTime &&
        muscleCamTriggerState) {
        muscleCamTriggerOff();
        muscleCamTriggerState = false;
    }

    // Should I execute the next protocol step?
    while (!protocolSteps.empty()) {
        ProtocolStep &currentStep = protocolSteps.front();
        if (currentStep.frameCount == behaviorTriggerCounter) {
            if (currentStep.operation == PROTOCOL_ENDS &&
                !behaviorTriggerState) {
                protocolSteps.clear();
                Serial.println("Protocol ended.");
                Serial.flush();
                litStatusLED(GREEN);
                // These are for checking if the last frame arrived as expected
                // (by making the following frames extra bright)
                // delayMicroseconds(10000);
                // behaviorTriggerOn();
                // delayMicroseconds(behaviorCamExposureTime * 10);
                // behaviorTriggerOff();
                // delayMicroseconds(10000);
                // behaviorTriggerOn();
                // delayMicroseconds(behaviorCamExposureTime * 10);
                // behaviorTriggerOff();
                // delayMicroseconds(10000);
                // behaviorTriggerOn();
                // delayMicroseconds(behaviorCamExposureTime * 10);
                // behaviorTriggerOff();
                // Stop triggering until reset by the computer-side program
                behaviorCamPeriod = 3600000000; // 1 hour
            } else {
                int pin;
                if (currentStep.optoChannel == 2) {
                    pin = OPTO_CH2_PIN;
                } else if (currentStep.optoChannel == 3) {
                    pin = OPTO_CH3_PIN;
                } else {
                    Serial.print("Invalid opto channel in protocol string: ");
                    Serial.println(currentStep.optoChannel);
                    Serial.flush();
                    litStatusLED(RED);
                    return;
                }
                digitalWrite(
                    pin, currentStep.operation == OPTO_ON ? HIGH : LOW);
                protocolSteps.erase(protocolSteps.begin());
            }
        } else {
            break;
        }
    }
}
