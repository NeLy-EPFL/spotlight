// On-device AUnit tests for the comm_protocol library.
//
// The desktop GoogleTest suite (comm_protocol/tests/) already covers the
// protocol logic thoroughly, but on a different toolchain. These run the same
// library on the actual Arduino Nano ESP32 to confirm it parses and serializes
// correctly there too -- in particular that ArduinoJson and the std::string /
// std::deque usage behave on-target. They are intentionally a smaller smoke
// suite rather than a port of every desktop case.

#include <AUnit.h>
#include <Arduino.h>

#include <deque>
#include <string>

#include <comm_protocol/protocol.h>

namespace {

// The Arduino Nano ESP32's native USB CDC reports "connected" immediately and
// is not reset when the port is opened, so the board would otherwise print its
// results before the test host (`pio test`) attaches and they would be lost.
// Delaying the start of the run leaves time for the host to open the port
// first. See the firmware test instructions in the project README.
constexpr unsigned long test_host_connect_delay_ms = 20000;

// TriggerParams whose fields satisfy every protocol constraint.
TriggerParams valid_params() {
    TriggerParams p;
    p.enable_muscle = false; // non-default, so round-trips must carry it
    p.beh_exp_time = 1000;
    p.musc_eff_exp_time = 2000;
    p.beh_frame_rate = 100;
    p.beh_musc_sync_ratio = 3;
    p.pco_cam_rolling_time = 50;
    p.pco_cam_readout_time = 80;
    return p;
}

} // namespace

test(protocol_streamRoundTrip) {
    Command original = Command::make_stream_command(valid_params());
    assertTrue(original.is_valid);

    std::string json = original.to_string();
    assertMore(json.size(), (size_t)0);

    Command parsed = Command::parse(json);
    assertTrue(parsed.is_valid);
    assertEqual((int)parsed.cmd_type, (int)CmdType::stream);
    assertEqual(parsed.params.enable_muscle, valid_params().enable_muscle);
    assertEqual(parsed.params.beh_frame_rate, valid_params().beh_frame_rate);
    assertEqual(parsed.params.beh_exp_time, valid_params().beh_exp_time);
}

test(protocol_startRecordingRoundTrip) {
    TriggerParams rec = valid_params();
    TriggerParams revert = valid_params();
    revert.beh_frame_rate = 50;
    revert.enable_muscle = true; // rec/revert may carry different modes

    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::ch2, OpType::on));
    seq.push_back(OperationStep(90, OptoChannel::all, OpType::stop));

    Command original = Command::make_start_recording_command(rec, revert, seq);
    assertTrue(original.is_valid);

    std::string json = original.to_string();
    assertMore(json.size(), (size_t)0);

    Command parsed = Command::parse(json);
    assertTrue(parsed.is_valid);
    assertEqual((int)parsed.cmd_type, (int)CmdType::start_recording);
    assertEqual(parsed.rec_params.enable_muscle, rec.enable_muscle);
    assertEqual(parsed.revert_to_params.enable_muscle, revert.enable_muscle);
    assertEqual(parsed.rec_params.beh_frame_rate, rec.beh_frame_rate);
    assertEqual(parsed.revert_to_params.beh_frame_rate, revert.beh_frame_rate);
    assertEqual(parsed.op_sequence.size(), (size_t)2);
    assertEqual((int)parsed.op_sequence[0].op, (int)OpType::on);
    assertEqual((int)parsed.op_sequence[1].op, (int)OpType::stop);
}

test(protocol_logRoundTrip) {
    Command original = Command::make_log_command("a log line");
    std::string json = original.to_string();

    Command parsed = Command::parse(json);
    assertTrue(parsed.is_valid);
    assertEqual((int)parsed.cmd_type, (int)CmdType::log);
    assertEqual(parsed.log_msg.c_str(), "a log line");
}

test(protocol_resetRoundTrip) {
    Command original = Command::make_reset_command();
    assertTrue(original.is_valid);

    std::string json = original.to_string();
    assertMore(json.size(), (size_t)0);

    Command parsed = Command::parse(json);
    assertTrue(parsed.is_valid);
    assertEqual((int)parsed.cmd_type, (int)CmdType::reset);
}

test(protocol_parseRejectsMalformed) {
    Command cmd = Command::parse("{not valid json");
    assertFalse(cmd.is_valid);
}

test(protocol_parseRejectsUnknownCmdType) {
    Command cmd = Command::parse(R"({"cmdType":"NOPE"})");
    assertFalse(cmd.is_valid);
}

test(protocol_operationStepValidity) {
    assertTrue(OperationStep(30, OptoChannel::ch2, OpType::on).is_valid);
    assertTrue(OperationStep(90, OptoChannel::all, OpType::stop).is_valid);
    // STOP must target ALL, not a single channel.
    assertFalse(OperationStep(90, OptoChannel::ch2, OpType::stop).is_valid);
    // ON/OFF must target a real opto channel, not ALL.
    assertFalse(OperationStep(30, OptoChannel::all, OpType::on).is_valid);
}

void setup() {
    Serial.begin(115200);
    // See the note in test_device_io on the host-connection delay.
    delay(test_host_connect_delay_ms);
    aunit::TestRunner::setTimeout(30);
}

void loop() {
    aunit::TestRunner::run();
}
