// Unit tests for the comm_protocol message library.
//
// These desktop tests use GoogleTest. They never run on the microcontroller
// (the firmware exercises the same library on-target through AUnit instead, see
// trigger_firmware/test/), so depending on a full desktop framework here is
// fine. GoogleTest is fetched automatically by the tests CMakeLists.

#include <comm_protocol/protocol.h>

#include <deque>
#include <string>

#include <gtest/gtest.h>

namespace {

/** A TriggerParams instance with values that satisfy every field constraint. */
TriggerParams validParams() {
    TriggerParams p;
    p.enable_muscle = false; // non-default, so round-trips must carry it
    p.beh_exp_time = 1000;
    p.musc_eff_exp_time = 2000;
    p.beh_frame_rate = 100;    // must be >= 1
    p.beh_musc_sync_ratio = 3; // must be >= 1
    p.pco_cam_rolling_time = 50;
    p.pco_cam_readout_time = 80;
    return p;
}

/** Field-by-field comparison of two TriggerParams, with per-field diagnostics.
 */
void expectParamsEqual(const TriggerParams &a, const TriggerParams &b) {
    EXPECT_EQ(a.enable_muscle, b.enable_muscle);
    EXPECT_EQ(a.beh_exp_time, b.beh_exp_time);
    EXPECT_EQ(a.musc_eff_exp_time, b.musc_eff_exp_time);
    EXPECT_EQ(a.beh_frame_rate, b.beh_frame_rate);
    EXPECT_EQ(a.beh_musc_sync_ratio, b.beh_musc_sync_ratio);
    EXPECT_EQ(a.pco_cam_rolling_time, b.pco_cam_rolling_time);
    EXPECT_EQ(a.pco_cam_readout_time, b.pco_cam_readout_time);
}

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

TEST(OperationStep, Validity) {
    // ON/OFF act on channel ch2 or ch3.
    EXPECT_TRUE(OperationStep(30, OptoChannel::ch2, OpType::on).is_valid);
    EXPECT_TRUE(OperationStep(0, OptoChannel::ch3, OpType::off).is_valid);
    EXPECT_FALSE(OperationStep(30, static_cast<OptoChannel>(1), OpType::on)
                     .is_valid); // channel 1 reserved for IR LED
    EXPECT_FALSE(OperationStep(30, static_cast<OptoChannel>(4), OpType::on)
                     .is_valid); // out of range
    EXPECT_FALSE(OperationStep(30, OptoChannel::all, OpType::off)
                     .is_valid); // all only for stop

    // STOP is global: channel all (-1). Any non-negative frame_idx is allowed.
    EXPECT_TRUE(OperationStep(90, OptoChannel::all, OpType::stop).is_valid);
    EXPECT_TRUE(OperationStep(0, OptoChannel::all, OpType::stop).is_valid);
    EXPECT_TRUE(OperationStep(91, OptoChannel::all, OpType::stop)
                    .is_valid); // frame_idx need not be a multiple of anything
    EXPECT_FALSE(OperationStep(90, OptoChannel::ch2, OpType::stop)
                     .is_valid); // channel must be all

    // A default-constructed step is invalid (built from no fields).
    EXPECT_FALSE(OperationStep().is_valid);
}

TEST(OperationStep, JsonRoundTrip) {
    OperationStep step(30, OptoChannel::ch2, OpType::on);
    ASSERT_TRUE(step.is_valid);

    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    step.to_json(obj);

    OperationStep parsed{JsonObjectConst(obj)};
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.frame_idx, step.frame_idx);
    EXPECT_EQ(parsed.channel, step.channel);
    EXPECT_EQ(parsed.op, step.op);
}

TEST(OperationStep, ParseRejectsBadOp) {
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    obj["frameIdx"] = 30;
    obj["channel"] = 2;
    obj["op"] = "WIGGLE"; // not a known op

    OperationStep parsed{JsonObjectConst(obj)};
    EXPECT_FALSE(parsed.is_valid);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

TEST(CommandBuilder, MakeStream) {
    Command cmd = Command::make_stream_command(validParams());
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::stream);
    expectParamsEqual(cmd.params, validParams());
}

TEST(CommandBuilder, MakeStopRecording) {
    Command cmd = Command::make_stop_recording_command();
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::stop_recording);
}

TEST(CommandBuilder, MakeLog) {
    Command cmd = Command::make_log_command("hello world");
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::log);
    EXPECT_EQ(cmd.log_msg, "hello world");
}

TEST(CommandBuilder, MakeReset) {
    Command cmd = Command::make_reset_command();
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::reset);
}

TEST(CommandBuilder, MakeStartRecordingValid) {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::ch2, OpType::on));
    seq.push_back(OperationStep(60, OptoChannel::ch3, OpType::off));
    seq.push_back(OperationStep(90, OptoChannel::all, OpType::stop));

    Command cmd = Command::make_start_recording_command(
        validParams(), validParams(), seq);
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::start_recording);
    EXPECT_EQ(cmd.op_sequence.size(), 3u);
}

TEST(CommandBuilder, MakeStartRecordingRejectsInvalidStep) {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::ch2, OpType::on));
    seq.push_back(
        OperationStep(60, OptoChannel::all, OpType::on)); // all invalid for on

    Command cmd = Command::make_start_recording_command(
        validParams(), validParams(), seq);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(CommandBuilder, MakeStartRecordingEmptySequence) {
    // An open recording carries an empty op_sequence and is still valid.
    Command cmd =
        Command::make_start_recording_command(validParams(), validParams(), {});
    EXPECT_TRUE(cmd.is_valid);
    EXPECT_TRUE(cmd.op_sequence.empty());
}

/* -------------------------------------------------------------------------- */
/* Serialization round trips                                                  */
/* -------------------------------------------------------------------------- */

TEST(RoundTrip, Stream) {
    Command original = Command::make_stream_command(validParams());
    std::string json = original.to_string();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.cmd_type, CmdType::stream);
    expectParamsEqual(parsed.params, original.params);
}

TEST(RoundTrip, StartRecording) {
    TriggerParams rec = validParams();
    TriggerParams revert = validParams();
    revert.beh_frame_rate = 50;  // distinguish the two param blocks
    revert.enable_muscle = true; // rec/revert may carry different modes

    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::ch2, OpType::on));
    seq.push_back(OperationStep(90, OptoChannel::all, OpType::stop));

    Command original = Command::make_start_recording_command(rec, revert, seq);
    std::string json = original.to_string();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.cmd_type, CmdType::start_recording);
    expectParamsEqual(parsed.rec_params, rec);
    expectParamsEqual(parsed.revert_to_params, revert);
    EXPECT_NE(
        parsed.rec_params.beh_frame_rate,
        parsed.revert_to_params.beh_frame_rate);
    EXPECT_NE(
        parsed.rec_params.enable_muscle, parsed.revert_to_params.enable_muscle);

    ASSERT_EQ(parsed.op_sequence.size(), 2u);
    EXPECT_EQ(parsed.op_sequence[0].frame_idx, 30u);
    EXPECT_EQ(parsed.op_sequence[0].channel, OptoChannel::ch2);
    EXPECT_EQ(parsed.op_sequence[0].op, OpType::on);
    EXPECT_EQ(parsed.op_sequence[1].frame_idx, 90u);
    EXPECT_EQ(parsed.op_sequence[1].channel, OptoChannel::all);
    EXPECT_EQ(parsed.op_sequence[1].op, OpType::stop);
}

TEST(RoundTrip, StopRecording) {
    Command original = Command::make_stop_recording_command();
    std::string json = original.to_string();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.cmd_type, CmdType::stop_recording);
}

TEST(RoundTrip, Reset) {
    Command original = Command::make_reset_command();
    std::string json = original.to_string();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.cmd_type, CmdType::reset);
}

TEST(RoundTrip, Log) {
    Command original = Command::make_log_command("a log line");
    std::string json = original.to_string();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.is_valid);
    EXPECT_EQ(parsed.cmd_type, CmdType::log);
    EXPECT_EQ(parsed.log_msg, "a log line");
}

/* -------------------------------------------------------------------------- */
/* Parsing failures                                                           */
/* -------------------------------------------------------------------------- */

TEST(Parse, MalformedJson) {
    Command cmd = Command::parse("{not valid json");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, UnknownCmdType) {
    Command cmd = Command::parse(R"({"cmdType":"NOPE"})");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, LegacyRunIsRejected) {
    // RUN was replaced by STREAM/START_RECORDING: it must no longer parse.
    Command cmd = Command::parse(R"({"cmdType":"RUN"})");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, MisspelledKeyIsRejected) {
    // The discriminator key is "cmdType", not the old "cmdTyp".
    Command cmd = Command::parse(R"({"cmdTyp":"LOG","msg":"x"})");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, LogMissingMsg) {
    Command cmd = Command::parse(R"({"cmdType":"LOG"})");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamMissingParams) {
    Command cmd = Command::parse(R"({"cmdType":"STREAM"})");
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamRejectsZeroFrameRate) {
    // behFrameRate must be strictly positive.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":true,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":0,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamRejectsMissingField) {
    // pcoCamReadoutTime is absent.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":true,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamRejectsMissingEnableMuscle) {
    // enableMuscle is a required field of every params block.
    std::string json =
        R"({"cmdType":"STREAM","params":{"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamRejectsNonBoolEnableMuscle) {
    // enableMuscle must be a JSON boolean, not a number or string.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":"yes","behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StreamAcceptsMuscleDisabled) {
    // A behavior-only params block (enableMuscle false) parses fine; the
    // muscle-only fields are still present but unused by the controller.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":false,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    ASSERT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::stream);
    EXPECT_FALSE(cmd.params.enable_muscle);
}

TEST(Parse, StartRecordingMissingRevertParams) {
    std::string json =
        R"({"cmdType":"START_RECORDING","recParams":{"enableMuscle":true,)"
        R"("behExpTime":0,"muscEffExpTime":0,"behFrameRate":100,)"
        R"("behMuscSyncRatio":3,"pcoCamRollingTime":0,)"
        R"("pcoCamReadoutTime":0},"opSequence":[]})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StartRecordingMissingOpSequence) {
    // opSequence is required even though it may be empty.
    std::string params =
        R"({"enableMuscle":true,"behExpTime":0,"muscEffExpTime":0,)"
        R"("behFrameRate":100,"behMuscSyncRatio":3,"pcoCamRollingTime":0,)"
        R"("pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params + "}";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StartRecordingRejectsBadStep) {
    // A STOP step on a specific channel (rather than ALL) is invalid.
    std::string params =
        R"({"enableMuscle":true,"behExpTime":0,"muscEffExpTime":0,)"
        R"("behFrameRate":100,"behMuscSyncRatio":3,"pcoCamRollingTime":0,)"
        R"("pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params +
                       R"(,"opSequence":[{"frameIdx":90,"channel":2,)"
                       R"("op":"STOP"}]})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.is_valid);
}

TEST(Parse, StartRecordingAcceptsEmptyOpSequence) {
    std::string params =
        R"({"enableMuscle":true,"behExpTime":0,"muscEffExpTime":0,)"
        R"("behFrameRate":100,"behMuscSyncRatio":3,"pcoCamRollingTime":0,)"
        R"("pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params +
                       R"(,"opSequence":[]})";
    Command cmd = Command::parse(json);
    ASSERT_TRUE(cmd.is_valid);
    EXPECT_EQ(cmd.cmd_type, CmdType::start_recording);
    EXPECT_TRUE(cmd.op_sequence.empty());
}

} // namespace
