// On-device AUnit tests for SerialIO's line framing.
//
// SerialIO reassembles newline-delimited messages out of a raw byte stream
// (CR tolerance, blank-line skipping, oversized-line dropping, and lines split
// across several update() calls). That logic is what the rest of the firmware
// relies on to never hand a truncated or partial command to Command::parse(),
// so it is worth testing directly.
//
// The real Serial port is busy carrying these very test results back to the
// host, so the tests cannot feed bytes through it. Instead they drive SerialIO
// from FakeStream, an in-memory Stream the test fills with bytes. SerialIO
// reads whichever Stream it was constructed with, so its framing logic is
// exercised exactly as on-target without touching the hardware port. These run
// on the board (uploaded by `pio test`, see the project README) only because
// that is where the firmware toolchain and Stream live.

#include <Arduino.h>
#include <AUnit.h>

#include <string>

#include "trigger_firmware/config.h"
#include "trigger_firmware/serial_io.h"

namespace {

// See the note in test_device_io on the host-connection delay.
constexpr unsigned long kTestHostConnectDelayMs = 20000;

// Minimal readable Arduino Stream backed by an in-memory queue of bytes. The
// test appends bytes with feed() (mimicking bytes arriving over the wire,
// possibly between update() calls) and SerialIO drains them via the Stream
// read interface. Writes are accepted and discarded so the Print base is
// satisfied; SerialIO never writes.
class FakeStream : public Stream {
  public:
    // Queue more incoming bytes, as if they had just arrived on the port.
    void feed(const std::string &bytes) { incoming_ += bytes; }

    int available() override {
        return static_cast<int>(incoming_.size() - pos_);
    }
    int read() override {
        if (pos_ >= incoming_.size()) {
            return -1;
        }
        return static_cast<unsigned char>(incoming_[pos_++]);
    }
    int peek() override {
        if (pos_ >= incoming_.size()) {
            return -1;
        }
        return static_cast<unsigned char>(incoming_[pos_]);
    }
    size_t write(uint8_t) override { return 1; }
    using Print::write;

  private:
    std::string incoming_;
    size_t pos_ = 0;
};

} // namespace

test(serialIo_returnsCompleteLineWithoutNewline) {
    FakeStream stream;
    SerialIO serial(stream);
    stream.feed("hello\n");

    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), "hello");
}

test(serialIo_noLineUntilNewlineArrives) {
    FakeStream stream;
    SerialIO serial(stream);

    // Nothing buffered yet.
    assertFalse((bool)serial.update());

    // A line with no terminator yet is still incomplete.
    stream.feed("partial");
    assertFalse((bool)serial.update());

    // The newline completes it.
    stream.feed("\n");
    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), "partial");
}

test(serialIo_reassemblesLineSplitAcrossUpdates) {
    FakeStream stream;
    SerialIO serial(stream);

    // A single logical line delivered in three chunks across three update()
    // calls is reassembled into one message.
    stream.feed("{\"cmd");
    assertFalse((bool)serial.update());
    stream.feed("Type\":\"L");
    assertFalse((bool)serial.update());
    stream.feed("OG\"}\n");

    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), R"({"cmdType":"LOG"})");
}

test(serialIo_stripsCarriageReturns) {
    FakeStream stream;
    SerialIO serial(stream);
    // A CRLF host: the trailing '\r' must not appear in the returned line.
    stream.feed("data\r\n");

    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), "data");
}

test(serialIo_skipsBlankLines) {
    FakeStream stream;
    SerialIO serial(stream);
    // Leading blank lines (including a bare CRLF) are skipped; update() returns
    // the first non-empty line.
    stream.feed("\n\r\nreal\n");

    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), "real");
}

test(serialIo_returnsOneMessagePerCall) {
    FakeStream stream;
    SerialIO serial(stream);
    // Three complete lines buffered at once must come back one per update().
    stream.feed("one\ntwo\nthree\n");

    std::optional<std::string> a = serial.update();
    std::optional<std::string> b = serial.update();
    std::optional<std::string> c = serial.update();
    assertTrue((bool)a);
    assertEqual(a->c_str(), "one");
    assertTrue((bool)b);
    assertEqual(b->c_str(), "two");
    assertTrue((bool)c);
    assertEqual(c->c_str(), "three");
    assertFalse((bool)serial.update());
}

test(serialIo_dropsOversizedLineButKeepsFollowing) {
    FakeStream stream;
    SerialIO serial(stream);

    // A line longer than the buffer must be discarded whole (never truncated
    // into a bogus "valid" message), and framing must recover for the next
    // line.
    std::string oversized(config::incomingCmdBufferSize + 50, 'a');
    stream.feed(oversized);
    stream.feed("\n");
    stream.feed("after\n");

    // The oversized line yields nothing...
    assertFalse((bool)serial.update());
    // ...and the following line is framed normally.
    std::optional<std::string> line = serial.update();
    assertTrue((bool)line);
    assertEqual(line->c_str(), "after");
}

void setup() {
    Serial.begin(config::serialBaudRate);
    // See the note in test_device_io on the host-connection delay.
    delay(kTestHostConnectDelayMs);
    aunit::TestRunner::setTimeout(30);
}

void loop() {
    aunit::TestRunner::run();
}
