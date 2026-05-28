#include <Arduino.h>

#include "pinAssignment.hpp"
#include "util.hpp"

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

bool updateCommandBuffer(char c, char *buffer, int &bufferIdx, int bufferSize) {
    if (c == '\n') {
        buffer[bufferIdx] = '\0';
        bufferIdx = 0;
        return true;
    } else if (bufferIdx < bufferSize - 1) {
        buffer[bufferIdx++] = c;
    }
    return false;
}
