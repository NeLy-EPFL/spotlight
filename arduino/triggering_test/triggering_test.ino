/*
 * Arduino Pin Timing Control
 *
 * This program:
 * - Turns pin 2 on at 100Hz, duration 1000us
 * - Turns pin 3 on at 100Hz, duration 1000us
 * - Turns pin 4 on every 4th time pin 2 is turned on, duration 3000us
 * - Turns pin 5 on every 4th time pin 2 is turned on, duration 3000us
 */

// Pin definitions
const int behCamPin = 2;
const int irLightPin = 3;
const int muscleCamPin = 4;
const int blueLightPin = 5;
const int optoCh1Pin = 6;
const int optoCh2Pin = 7;

int behCycleInterval = 1000000 / 150;
int behExposureTime = 1000;
int muscleBehMultiple = 5;
int muscleExposureTime = 3000;

unsigned long lastBehTriggerStartTime = 0;
unsigned long behaviorTriggerCycleCount = 0;
bool isBehTriggerOn = false;
bool isMuscleTriggerOn = false;

void setup() {
    // Initialize all pins as outputs
    pinMode(behCamPin, OUTPUT);
    pinMode(irLightPin, OUTPUT);
    pinMode(muscleCamPin, OUTPUT);
    pinMode(blueLightPin, OUTPUT);
    pinMode(optoCh1Pin, OUTPUT);
    pinMode(optoCh2Pin, OUTPUT);

    // Ensure all pins start in the OFF state
    digitalWrite(behCamPin, LOW);
    digitalWrite(irLightPin, LOW);
    digitalWrite(muscleCamPin, LOW);
    digitalWrite(blueLightPin, LOW);
    digitalWrite(optoCh1Pin, LOW);
    digitalWrite(optoCh2Pin, LOW);

    // Start serial comm
    Serial.begin(9600);
}

void loop() {
    unsigned long currentTime = micros();
    unsigned long timeSinceLastTriggerStart =
        currentTime - lastBehTriggerStartTime;

    // Turn behavior trigger off if exposure time has passed
    if (timeSinceLastTriggerStart >= behExposureTime) {
        if (isBehTriggerOn) {
            isBehTriggerOn = false;
            digitalWrite(behCamPin, LOW);
            digitalWrite(irLightPin, LOW);
        }
    }

    // Turn muscle trigger off if exposure time has passed
    if (timeSinceLastTriggerStart >= muscleExposureTime) {
        if (isMuscleTriggerOn) {
            isMuscleTriggerOn = false;
            digitalWrite(muscleCamPin, LOW);
            digitalWrite(blueLightPin, LOW);
        }
    }

    // Turn behavior trigger on for the next round
    if (timeSinceLastTriggerStart >= behCycleInterval) {
        if (isBehTriggerOn) {
            Serial.println("Error: Behavior trigger shouldn't be on here!");
        }

        isBehTriggerOn = true;
        digitalWrite(behCamPin, HIGH);
        digitalWrite(irLightPin, HIGH);

        if (behaviorTriggerCycleCount % muscleBehMultiple == 0) {
            isMuscleTriggerOn = true;
            digitalWrite(muscleCamPin, HIGH);
            digitalWrite(blueLightPin, HIGH);
        }

        lastBehTriggerStartTime = currentTime;
        ++behaviorTriggerCycleCount;
    }
}