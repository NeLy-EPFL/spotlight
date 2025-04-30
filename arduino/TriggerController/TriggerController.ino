#include <Arduino.h>

#include <string>
#include <vector>

#include "experimentProtocol.hpp"

// Pin assignments
const int BEHAVIOR_CAM_PIN = 2;
const int IR_LIGHT_PIN = 3;
const int MUSCLE_CAM_PIN = 4;
const int BLUE_LIGHT_PIN = 5;
const int OPTO_CH2_PIN = 6;
const int OPTO_CH3_PIN = 7;
const int GREEN_LIGHT_PIN = 8;
const int STATUS_LED_RED_PIN = 10;
const int STATUS_LED_GREEN_PIN = 11;
const int STATUS_LED_BLUE_PIN = 12;

const int MAX_PROGRAM_STEPS = 1024;
const int INCOMING_MESSAGE_BUFFER_SIZE = 16384;

const int FRAME_BUFFER_FLUSH_TIME_US = 100000;

// Default parameters
const unsigned int defaultBehaviorCamPeriod = 1000000 / 25; // in microseconds
const unsigned int defaultSyncRatioK = 1; // k behavior triggers per muscle trigger

volatile unsigned int behaviorCamPeriod = defaultBehaviorCamPeriod;
volatile unsigned int syncRatioK = defaultSyncRatioK;
volatile unsigned int behaviorCamExposureTime = 1000; // in microseconds
volatile unsigned int muscleCamExposureTime = 3000; // in microseconds
std::vector<ProtocolStep> protocolSteps;

// Timing variables
volatile unsigned long lastBehaviorTriggerTime = 0;
volatile unsigned long lastMuscleTriggerTime = 0;
volatile unsigned long behaviorTriggerCounter = 0;
volatile bool behaviorTriggerState = false;
volatile bool muscleTriggerState = false;

char incomingMessageBuffer[INCOMING_MESSAGE_BUFFER_SIZE];
int incomingMessageBufferIdx = 0;

enum Color {
  RED,
  GREEN,
  BLUE,
  MAGENTA,
  CYAN,
  YELLOW,
  WHITE,
  OFF,
};

void behaviorTriggerOn() {
  digitalWrite(BEHAVIOR_CAM_PIN, HIGH);
  digitalWrite(IR_LIGHT_PIN, HIGH);
}

void behaviorTriggerOff() {
  digitalWrite(BEHAVIOR_CAM_PIN, LOW);
  digitalWrite(IR_LIGHT_PIN, LOW);
}

void muscleTriggerOn() {
  digitalWrite(MUSCLE_CAM_PIN, HIGH);
  digitalWrite(BLUE_LIGHT_PIN, HIGH);
}

void muscleTriggerOff() {
  digitalWrite(MUSCLE_CAM_PIN, LOW);
  digitalWrite(BLUE_LIGHT_PIN, LOW);
}

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
  muscleTriggerOff();
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
    
    // Handle end of command (newline)
    if (c == '\n') {
      incomingMessageBuffer[incomingMessageBufferIdx] = '\0';  // Null-terminate the string
      filterAndProcessIncomingMessage(incomingMessageBuffer);
      incomingMessageBufferIdx = 0;  // Reset buffer index
    } 
    // Add character to buffer if there's space
    else if (incomingMessageBufferIdx < INCOMING_MESSAGE_BUFFER_SIZE - 1) {
      incomingMessageBuffer[incomingMessageBufferIdx++] = c;
    }
  }

  unsigned long currentTime = micros();

  // Should I open behavior camera shutter?
  if (currentTime - lastBehaviorTriggerTime >= behaviorCamPeriod &&
      !behaviorTriggerState) {
    behaviorTriggerOn();
    behaviorTriggerState = true;
    lastBehaviorTriggerTime = currentTime;

    // Should I also open muscle camera shutter?
    if (behaviorTriggerCounter % syncRatioK == 0) {
      muscleTriggerOn();
      muscleTriggerState = true;
      lastMuscleTriggerTime = currentTime;
    }

    behaviorTriggerCounter++;
  }

  // Should I close behavior camera shutter?
  if (currentTime - lastBehaviorTriggerTime >= behaviorCamExposureTime &&
      behaviorTriggerState) {
    behaviorTriggerOff();
    behaviorTriggerState = false;
  }
  
  // Should I close muscle camera shutter?
  if (currentTime - lastMuscleTriggerTime >= muscleCamExposureTime &&
      muscleTriggerState) {
    muscleTriggerOff();
    muscleTriggerState = false;
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
        digitalWrite(pin, currentStep.operation == OPTO_ON ? HIGH : LOW);
        protocolSteps.erase(protocolSteps.begin());
      }
    } else {
      break;
    }
  }
}

void filterAndProcessIncomingMessage(const char* message) {
  if (message[0] == '>') {
    parseIncomingCommand(message);
  } else {
    Serial.print("Arduino received the following message: '");
    Serial.print(message);
    Serial.println("' by serial comm.");
    Serial.flush();
  }
}

void parseIncomingCommand(const char* message) {
  if (strncmp(message, CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME, CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME) == 0) {
    // Handle command: SET_BEHAVIOR_EXPOSURE_TIME
    const char* expTimeStart = message + CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME + 1;
    unsigned int expTime = atoi(expTimeStart);
    if (expTime > 0) {
      behaviorCamExposureTime = expTime;
      Serial.print("Behavior camera exposure time set to: ");
      Serial.println(behaviorCamExposureTime);
      Serial.flush();
      // litStatusLED(GREEN);
    } else {
      Serial.println("Invalid exposure time for behavior camera: " + String(expTime));
      Serial.flush();
      litStatusLED(RED);
    }
  } else if (strncmp(message, CMDSTR_SET_MUSCLE_EXPOSURE_TIME, CMDLEN_SET_MUSCLE_EXPOSURE_TIME) == 0) {
    // Handle command: SET_MUSCLE_EXPOSURE_TIME
    const char* expTimeStart = message + CMDLEN_SET_MUSCLE_EXPOSURE_TIME + 1;
    unsigned int expTime = atoi(expTimeStart);
    if (expTime > 0) {
      muscleCamExposureTime = expTime;
      Serial.print("Muscle camera exposure time set to: ");
      Serial.println(muscleCamExposureTime);
      Serial.flush();
      // litStatusLED(GREEN);
    } else {
      Serial.print("Invalid exposure time for muscle camera: ");
      Serial.println(expTime);
      Serial.flush();
      litStatusLED(RED);
    }
  } else if (strncmp(message, CMDSTR_SET_BEHAVIOR_FPS, CMDLEN_SET_BEHAVIOR_FPS) == 0) {
    // Handle command: SET_BEHAVIOR_FPS
    const char* fpsStart = message + CMDLEN_SET_BEHAVIOR_FPS + 1;
    unsigned int fps = atoi(fpsStart);
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
  } else if (strncmp(message, CMDSTR_SET_SYNC_RATIO, CMDLEN_SET_SYNC_RATIO) == 0) {
    // Handle command: SET_SYNC_RATIO
    const char* ratioStart = message + CMDLEN_SET_SYNC_RATIO + 1;
    unsigned int ratio = atoi(ratioStart);
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
  } else if (strncmp(message, CMDSTR_START_RECORDING, CMDLEN_START_RECORDING) == 0) {
    // Handle command: START_RECORDING
    const char* protocolStart = message + CMDLEN_START_RECORDING + 1;
    int numStepsParsed = parseProtocolSequence(protocolStart, protocolSteps);
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
    lastMuscleTriggerTime = 0;
    behaviorTriggerState = false;
    muscleTriggerState = false;
    Serial.println("Pausing to let frame buffer clear...");
    Serial.flush();
    behaviorTriggerOff();
    muscleTriggerOff();
    delayMicroseconds(FRAME_BUFFER_FLUSH_TIME_US);
    Serial.println("Recording started.");
    Serial.flush();
  } else if (strncmp(message, CMDSTR_STOP_RECORDING, CMDLEN_STOP_RECORDING) == 0) {
    // Handle command: STOP_RECORDING
    protocolSteps.clear();
    behaviorTriggerCounter = 0;
    Serial.println("Recording stopped.");
    Serial.flush();
  } else {
    Serial.print("Arduino received the following command: '");
    Serial.print(message);
    Serial.println("' by serial comm.");
    Serial.flush();
    litStatusLED(RED);
  }
}

void litStatusLED(Color color) {
  switch (color) {
    case RED:
    analogWrite(STATUS_LED_RED_PIN, 255);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case GREEN:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 255);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case BLUE:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 255);
    break;
    case MAGENTA:
    analogWrite(STATUS_LED_RED_PIN, 128);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 128);
    break;
    case CYAN:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 128);
    analogWrite(STATUS_LED_BLUE_PIN, 128);
    break;
    case YELLOW:
    analogWrite(STATUS_LED_RED_PIN, 128);
    analogWrite(STATUS_LED_GREEN_PIN, 128);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case WHITE:
    analogWrite(STATUS_LED_RED_PIN, 85);
    analogWrite(STATUS_LED_GREEN_PIN, 85);
    analogWrite(STATUS_LED_BLUE_PIN, 85);
    break;
    case OFF:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
  }
}