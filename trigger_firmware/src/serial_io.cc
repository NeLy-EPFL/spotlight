#include "trigger_firmware/serial_io.h"

#include <Arduino.h>

#include "trigger_firmware/config.h"

void SerialIO::begin() {
    Serial.begin(config::serialBaudRate);
}

std::optional<std::string> SerialIO::update() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // Tolerate CR (e.g. a "\r\n" host); treat LF as the line terminator.
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            bool overflowed = overflowed_;
            int length = bufferIdx_;
            bufferIdx_ = 0;
            overflowed_ = false;

            // Drop oversized lines and ignore blank lines; only hand back a
            // complete, non-empty line.
            if (overflowed || length == 0) {
                continue;
            }
            return std::string(buffer_, length);
        }

        if (bufferIdx_ < config::incomingCmdBufferSize - 1) {
            buffer_[bufferIdx_] = c;
            bufferIdx_++;
        } else {
            overflowed_ = true;
        }
    }
    return std::nullopt;
}
