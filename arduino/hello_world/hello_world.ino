#define LED_PIN 9

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(250);  // ms
    digitalWrite(LED_PIN, LOW);
    delay(250);  // ms
}
