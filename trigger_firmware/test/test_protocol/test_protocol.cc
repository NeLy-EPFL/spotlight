// On-device AUnit tests for the comm_protocol library.
//
// The desktop GoogleTest suite (comm_protocol/tests/) already covers the
// protocol logic thoroughly, but on a different toolchain. These run the same
// library on the actual Arduino Nano ESP32 to confirm it parses and serializes
// correctly there too -- in particular that ArduinoJson and the std::string /
// std::deque usage behave on-target. They are intentionally a smaller smoke
// suite rather than a port of every desktop case.

#include <Arduino.h>
#include <AUnit.h>

#include <deque>
#include <string>

#include <comm_protocol/protocol.h>

namespace {

// The Arduino Nano ESP32's native USB CDC reports "connected" immediately and
// is not reset when the port is opened, so the board would otherwise print its
// results before the test host (`pio test`) attaches and they would be lost.
// Delaying the start of the run leaves time for the host to open the port first.
// See the firmware test instructions in the project README.
constexpr unsigned long kTestHostConnectDelayMs = 20000;

// TriggerParams whose fields satisfy every protocol constraint.
TriggerParams validParams() {
    TriggerParams p;
    p.behExpTime = 1000;
    p.muscEffExpTime = 2000;
    p.behFrameRate = 100;
    p.behMuscSyncRatio = 3;
    p.pcoCamRollingTime = 50;
    p.pcoCamReadoutTime = 80;
    return p;
}

} // namespace

test(protocol_streamRoundTrip) {
    Command original = Command::makeStreamCommand(validParams());
    assertTrue(original.isValid);

    std::string json = original.toString();
    assertMore(json.size(), (size_t)0);

    Command parsed = Command::parse(json);
    assertTrue(parsed.isValid);
    assertEqual((int)parsed.cmdType, (int)CmdType::STREAM);
    assertEqual(parsed.params.behFrameRate, validParams().behFrameRate);
    assertEqual(parsed.params.behExpTime, validParams().behExpTime);
}

test(protocol_startRecordingRoundTrip) {
    TriggerParams rec = validParams();
    TriggerParams revert = validParams();
    revert.behFrameRate = 50;

    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(OperationStep(90, OptoChannel::ALL, OpType::STOP));

    Command original = Command::makeStartRecordingCommand(rec, revert, seq);
    assertTrue(original.isValid);

    std::string json = original.toString();
    assertMore(json.size(), (size_t)0);

    Command parsed = Command::parse(json);
    assertTrue(parsed.isValid);
    assertEqual((int)parsed.cmdType, (int)CmdType::START_RECORDING);
    assertEqual(parsed.recParams.behFrameRate, rec.behFrameRate);
    assertEqual(parsed.revertToParams.behFrameRate, revert.behFrameRate);
    assertEqual(parsed.opSequence.size(), (size_t)2);
    assertEqual((int)parsed.opSequence[0].op, (int)OpType::ON);
    assertEqual((int)parsed.opSequence[1].op, (int)OpType::STOP);
}

test(protocol_logRoundTrip) {
    Command original = Command::makeLogCommand("a log line");
    std::string json = original.toString();

    Command parsed = Command::parse(json);
    assertTrue(parsed.isValid);
    assertEqual((int)parsed.cmdType, (int)CmdType::LOG);
    assertEqual(parsed.logMsg.c_str(), "a log line");
}

test(protocol_parseRejectsMalformed) {
    Command cmd = Command::parse("{not valid json");
    assertFalse(cmd.isValid);
}

test(protocol_parseRejectsUnknownCmdType) {
    Command cmd = Command::parse(R"({"cmdType":"NOPE"})");
    assertFalse(cmd.isValid);
}

test(protocol_operationStepValidity) {
    assertTrue(OperationStep(30, OptoChannel::CH2, OpType::ON).isValid);
    assertTrue(OperationStep(90, OptoChannel::ALL, OpType::STOP).isValid);
    // STOP must target ALL, not a single channel.
    assertFalse(OperationStep(90, OptoChannel::CH2, OpType::STOP).isValid);
    // ON/OFF must target a real opto channel, not ALL.
    assertFalse(OperationStep(30, OptoChannel::ALL, OpType::ON).isValid);
}

void setup() {
    Serial.begin(115200);
    // See the note in test_device_io on the host-connection delay.
    delay(kTestHostConnectDelayMs);
    aunit::TestRunner::setTimeout(30);
}

void loop() {
    aunit::TestRunner::run();
}
