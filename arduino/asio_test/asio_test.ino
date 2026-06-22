void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);
}

void loop() {
    // Check if data is available to read
    if (Serial.available() > 0) {
        // Read the incoming string
        String received = Serial.readStringUntil('\n');

        // Send a response back
        Serial.print("Arduino received: ");
        Serial.println(received);
    }
}