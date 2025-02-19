// Arduino Code
const int triggerPin = 2;
const int irIlluminationPin = 3;
const int indicatorPin = 6;
int blinkIntervalSlow = 1000 / 50;
int blinkIntervalFast = 1000 / 100;

unsigned long lastBlinkTime = 0;
bool triggerState = LOW;
int halfInterval = 1000 / 100;


void setup() {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(indicatorPin, triggerState);
    pinMode(indicatorPin, OUTPUT);
    pinMode(irIlluminationPin, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // Check for serial input
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "START") {
            halfInterval = blinkIntervalFast / 2;
        } else if (command == "STOP") {
            halfInterval = blinkIntervalSlow / 2;
        }
    }
    
    // Blink logic
    if (millis() - lastBlinkTime >= halfInterval) {
        lastBlinkTime = millis();
        triggerState = !triggerState;
        digitalWrite(triggerPin, triggerState);
        digitalWrite(indicatorPin, triggerState);
        digitalWrite(irIlluminationPin, triggerState);
    }
}

