#ifndef UTIL_HPP
#define UTIL_HPP

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

// Set the RGB status LED to the given color.
void litStatusLED(Color color);

// Append one character to the incoming message buffer.
// Returns true when a complete (newline-terminated) message has been received;
// on return the buffer is null-terminated and bufferIdx has been reset to 0.
bool updateCommandBuffer(char c, char *buffer, int &bufferIdx, int bufferSize);

#endif
