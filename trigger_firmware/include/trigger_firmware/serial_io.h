#pragma once

#include <optional>
#include <string>

#include "trigger_firmware/config.h"

// Arduino input/output stream base class. Forward-declared (instead of pulling
// in <Arduino.h> here) because the header only needs the reference type.
class Stream;

/**
 * Line-framing layer over the USB serial port.
 *
 * The recorder sends each protocol message (see docs/comm_protocol.md) as a
 * single compact JSON object terminated by a newline. This class reassembles
 * those newline-delimited lines from the raw byte stream; it does not interpret
 * them. The controller passes the returned line to Command::parse() to decode
 * the actual command (STREAM, START_RECORDING, STOP_RECORDING, or LOG).
 *
 * Usage: call begin() once during setup(), then call update() at the top of
 * every controller cycle. update() drains whatever bytes have arrived and, when
 * a complete line has been received, returns it. Because a line may span
 * several cycles (or several lines may be buffered at once), update() returns
 * at most one message per call and keeps any partial/trailing bytes for the
 * next call.
 *
 * The input stream defaults to the global `Serial` (the USB CDC port used in
 * production). A different Stream can be injected via the constructor so the
 * framing logic can be unit-tested against an in-memory stream without using
 * the real serial port -- which on-device is busy carrying the test results
 * (see test/test_serial_io/).
 */
class SerialIO {
  public:
    /** Read from the global `Serial` (the production USB CDC port). */
    SerialIO();
    /** Read from an injected stream; used by the unit tests. */
    explicit SerialIO(Stream &stream);

    /** Open the underlying USB serial port at config::serialBaudRate. */
    void begin();

    /**
     * Consume newly arrived bytes and return the next complete line, if any.
     *
     * Returns std::nullopt when no full line is available yet. The returned
     * string excludes the terminating newline. Blank lines are skipped, and a
     * line longer than config::incomingCmdBufferSize is discarded rather than
     * truncated (so a malformed/oversized message is never reported as valid).
     */
    std::optional<std::string> update();

  private:
    Stream &stream_;
    char buffer_[config::incomingCmdBufferSize];
    int bufferIdx_ = 0;
    // Set once the current line overruns buffer_; the rest of the line (up to
    // the next newline) is then dropped instead of silently truncated.
    bool overflowed_ = false;
};
