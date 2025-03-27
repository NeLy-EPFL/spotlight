/**
 * @file TriggerController.ino
 * @brief Arduino program for camera and light triggering.
 */

#include "parserAndFormatter.hpp"

// Pin assignments
const int BEHAVIOR_CAM_PIN = 2;
const int IR_LIGHT_PIN = 3;
const int MUSCLE_CAM_PIN = 4;
const int BLUE_LIGHT_PIN = 5;
const int OPTO_CH1_PIN = 6;
const int OPTO_CH2_PIN = 7;
const int GREEN_LIGHT_PIN = 8;
const int STATUS_LED_RED_PIN = 10;
const int STATUS_LED_GREEN_PIN = 11;
const int STATUS_LED_BLUE_PIN = 12;

const int MAX_PROGRAM_STEPS = 1024;
const int COMMAND_BUFFER_SIZE = 16384;

// Default parameters
volatile unsigned int behaviorCamPeriod = 1000000 / 25; // in microseconds
volatile unsigned int syncRatioK = 1; // k behavior triggers per muscle trigger
volatile unsigned int behaviorCamExposureTime = 1000; // in microseconds
volatile unsigned int muscleCamExposureTime = 3000; // in microseconds

// Control flags
volatile bool isTriggering = false;
volatile bool hasProgram = false;

// Timing variables
volatile unsigned long lastBehaviorTriggerTime = 0;
volatile unsigned long lastMuscleTriggerTime = 0;
volatile unsigned long behaviorTriggerCounter = 0;
volatile bool behaviorTriggerState = false;
volatile bool muscleTriggerState = false;

// Program for device 3
char cmdBuffer[COMMAND_BUFFER_SIZE];
int cmdIndex = 0;

volatile ProtocolStep programSteps[MAX_PROGRAM_STEPS];
volatile int numProtocolSteps = 0;
volatile int currentProtocolStep = 0;
volatile unsigned long programStartTime = 0;

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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Set pin modes
  pinMode(BEHAVIOR_CAM_PIN, OUTPUT);
  pinMode(IR_LIGHT_PIN, OUTPUT);
  pinMode(MUSCLE_CAM_PIN, OUTPUT);
  pinMode(BLUE_LIGHT_PIN, OUTPUT);
  pinMode(OPTO_CH1_PIN, OUTPUT);
  pinMode(OPTO_CH2_PIN, OUTPUT);
  pinMode(GREEN_LIGHT_PIN, OUTPUT);
  pinMode(STATUS_LED_RED_PIN, OUTPUT);
  pinMode(STATUS_LED_GREEN_PIN, OUTPUT);
  pinMode(STATUS_LED_BLUE_PIN, OUTPUT);
  
  // Initialize pins to LOW
  behaviorTriggerOff();
  muscleTriggerOff();
  digitalWrite(OPTO_CH1_PIN, LOW);
  digitalWrite(OPTO_CH2_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN, LOW);
  digitalWrite(STATUS_LED_RED_PIN, LOW);
  digitalWrite(STATUS_LED_GREEN_PIN, LOW);
  digitalWrite(STATUS_LED_BLUE_PIN, LOW);
}

void loop() {
  // Check for incoming commands
  readAndProcessSerialCommand();
  
  // If triggering is active, handle camera triggers
  if (isTriggering) {
    unsigned long currentTime = micros();
    
    // Is it time to turn the triggers on?
    if ((unsigned long)(currentTime - lastBehaviorTriggerTime) >= behaviorCamPeriod) {
      if (!behaviorTriggerState) {
        behaviorTriggerOn();
        behaviorTriggerState = true;
        behaviorTriggerCounter++;
      }
      
      // Should we also trigger the muscle camera this cycle?
      if ((behaviorTriggerCounter % syncRatioK == 0) && (!muscleTriggerState)) {
        muscleTriggerOn();
        muscleTriggerState = true;
        lastMuscleTriggerTime = currentTime;
      }
      
      lastBehaviorTriggerTime = currentTime;
    }

    // Is it time to turn the triggers off?
    if ((unsigned long)(currentTime - lastBehaviorTriggerTime) >= behaviorCamExposureTime) {
      if (behaviorTriggerState) {
        behaviorTriggerOff();
        behaviorTriggerState = false;
      }
    }
    if ((unsigned long)(currentTime - lastMuscleTriggerTime) >= muscleCamExposureTime) {
      if (muscleTriggerState) {
        muscleTriggerOff();
        muscleTriggerState = false;
      }
    }
    
    // Handle preset protocol if there is one
    if (hasProgram && currentProtocolStep < numProtocolSteps) {
      if (behaviorTriggerCounter == programSteps[currentProtocolStep].frameCount) {
        if (programSteps[currentProtocolStep].isDone) {
          stopAllTriggering();
          numProtocolSteps = 0;
          currentProtocolStep = 0;
          hasProgram = false;
          litStatusLED(WHITE);
          Serial.println("PROGRAM_ENDS");
        } else {
          int pin;
          if (programSteps[currentProtocolStep].channel == 1) {
            pin = OPTO_CH1_PIN;
          } else if (programSteps[currentProtocolStep].channel == 2) {
            pin = OPTO_CH2_PIN;
          } else {
            // `channel` shouldn't be -1 here because it is -1 only if the step
            // is "stop", which is handled above.
            Serial.println("ERROR: Invalid channel number in protocol; cannot continue.");
            stopAllTriggering();
            litStatusLED(RED);
            return;
          }
          int signal = programSteps[currentProtocolStep].operation == OPTO_ON ? HIGH : LOW;
          digitalWrite(pin, signal);
          currentProtocolStep++;
        }
      }
    }
  }
}

/**
 * @brief Reads and processes serial commands from the Serial input buffer.
 * 
 * This function checks the Serial input buffer for available data. It reads
 * characters one by one and appends them to a command buffer. When a newline 
 * character ('\n') is encountered, it treats the accumulated characters as a
 * complete command, null-terminates the string, and passes it to the 
 * `processCommand` function for further handling. The command buffer index is 
 * then reset for the next command.
 * 
 * @note Ensure that `cmdBuffer` and `cmdIndex` are properly defined and 
 * initialized in the global scope. The size of `cmdBuffer` determines the 
 * maximum length of a command that can be processed.
 */
void readAndProcessSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Handle end of command (newline)
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';  // Null-terminate the string
      processCommand(cmdBuffer);
      cmdIndex = 0;  // Reset buffer index
    } 
    // Add character to buffer if there's space
    else if (cmdIndex < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

void processCommand(const char* command) {
  // SET_BEHAVIOR_FREQ command
  if (strncmp(command,
              CMDSTR_SET_BEHAVIOR_PERIOD,
              CMDLEN_SET_BEHAVIOR_PERIOD) == 0) {
    behaviorCamPeriod = atoi(command + CMDLEN_SET_BEHAVIOR_PERIOD + 1);
    Serial.print(CMDSTR_SET_BEHAVIOR_PERIOD);
    Serial.print(ACK_SUFFIX);
    Serial.print(" ");
    Serial.println(behaviorCamPeriod);
  }
  // SET_SYNC_RATIO command
  else if (strncmp(command,
                   CMDSTR_SET_SYNC_RATIO,
                   CMDLEN_SET_SYNC_RATIO) == 0) {
    int ratio = atoi(command + CMDLEN_SET_SYNC_RATIO + 1);
    syncRatioK = ratio;
    Serial.print(CMDSTR_SET_SYNC_RATIO);
    Serial.print(ACK_SUFFIX);
    Serial.print(" ");
    Serial.println(ratio);
  }
  // SET_BEHAVIOR_EXPOSURE_TIME command
  else if (strncmp(command,
                   CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME,
                   CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME) == 0) {
    int duration = atoi(command + CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME + 1);
    behaviorCamExposureTime = duration;
    Serial.print(CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME);
    Serial.print(ACK_SUFFIX);
    Serial.print(" ");
    Serial.println(duration);
  }
  // SET_MUSCLE_EXPOSURE_TIME command
  else if (strncmp(command,
                   CMDSTR_SET_MUSCLE_EXPOSURE_TIME,
                   CMDLEN_SET_MUSCLE_EXPOSURE_TIME) == 0) {
    unsigned long duration = atoi(command + CMDLEN_SET_MUSCLE_EXPOSURE_TIME + 1);
    muscleCamExposureTime = duration;
    Serial.print(CMDSTR_SET_MUSCLE_EXPOSURE_TIME);
    Serial.print(ACK_SUFFIX);
    Serial.print(" ");
    Serial.println(duration);
  }
  // START_TRIGGERING command
  else if (strncmp(command,
                   CMDSTR_START_TRIGGERING,
                   CMDLEN_START_TRIGGERING) == 0) {
    const char* programStr = command + CMDLEN_START_TRIGGERING + 1;
    numProtocolSteps = parseEntireProtocol(
      programStr, programSteps, MAX_PROGRAM_STEPS);
    if (numProtocolSteps < 0) {
      Serial.println("ERROR: Invalid protocol format; cannot start recording.");
      hasProgram = false;
      litStatusLED(RED);
      return;
    } else if (numProtocolSteps == 0) {
      Serial.println("Arduino received empty protocol; running openly.");
      hasProgram = false;
      litStatusLED(BLUE);
    } else {
      Serial.print("Arduino received valid protocol; recording accordingly.");
      hasProgram = true;
      litStatusLED(MAGENTA);
    }
    isTriggering = true;
    behaviorTriggerCounter = 0;
    lastBehaviorTriggerTime = micros();
    programStartTime = micros();
    currentProtocolStep = 0;
    Serial.print(CMDSTR_START_TRIGGERING);
    Serial.print(ACK_SUFFIX);
    Serial.print(" ");
    Serial.println(programStr);
  }
  // STOP_TRIGGERING command
  else if (strcmp(command, CMDSTR_STOP_TRIGGERING) == 0) {
    stopAllTriggering();
    litStatusLED(WHITE);
    hasProgram = false;
    numProtocolSteps = 0;
    currentProtocolStep = 0;
    Serial.print(CMDSTR_STOP_TRIGGERING);
    Serial.println(ACK_SUFFIX);
  }
}

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

void stopAllTriggering() {
  isTriggering = false;
  hasProgram = false;
  digitalWrite(BEHAVIOR_CAM_PIN, LOW);
  digitalWrite(MUSCLE_CAM_PIN, LOW);
  digitalWrite(OPTO_CH1_PIN, LOW);
  digitalWrite(OPTO_CH2_PIN, LOW);
}

void litStatusLED(Color color) {
  switch (color) {
    case RED:
    analogWrite(STATUS_LED_RED_PIN, 180);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case GREEN:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 180);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case BLUE:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 180);
    break;
    case MAGENTA:
    analogWrite(STATUS_LED_RED_PIN, 90);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 90);
    break;
    case CYAN:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 90);
    analogWrite(STATUS_LED_BLUE_PIN, 90);
    break;
    case YELLOW:
    analogWrite(STATUS_LED_RED_PIN, 90);
    analogWrite(STATUS_LED_GREEN_PIN, 90);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
    case WHITE:
    analogWrite(STATUS_LED_RED_PIN, 60);
    analogWrite(STATUS_LED_GREEN_PIN, 60);
    analogWrite(STATUS_LED_BLUE_PIN, 60);
    break;
    case OFF:
    analogWrite(STATUS_LED_RED_PIN, 0);
    analogWrite(STATUS_LED_GREEN_PIN, 0);
    analogWrite(STATUS_LED_BLUE_PIN, 0);
    break;
  }
}