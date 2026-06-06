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
    p.enableMuscle = false; // non-default, so round-trips must carry it
    p.behExpTime = 1000;
    p.muscEffExpTime = 2000;
    p.behFrameRate = 100;   // must be >= 1
    p.behMuscSyncRatio = 3; // must be >= 1
    p.pcoCamRollingTime = 50;
    p.pcoCamReadoutTime = 80;
    return p;
}

/** Field-by-field comparison of two TriggerParams, with per-field diagnostics. */
void expectParamsEqual(const TriggerParams &a, const TriggerParams &b) {
    EXPECT_EQ(a.enableMuscle, b.enableMuscle);
    EXPECT_EQ(a.behExpTime, b.behExpTime);
    EXPECT_EQ(a.muscEffExpTime, b.muscEffExpTime);
    EXPECT_EQ(a.behFrameRate, b.behFrameRate);
    EXPECT_EQ(a.behMuscSyncRatio, b.behMuscSyncRatio);
    EXPECT_EQ(a.pcoCamRollingTime, b.pcoCamRollingTime);
    EXPECT_EQ(a.pcoCamReadoutTime, b.pcoCamReadoutTime);
}

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

TEST(OperationStep, Validity) {
    // ON/OFF act on channel CH2 or CH3.
    EXPECT_TRUE(OperationStep(30, OptoChannel::CH2, OpType::ON).isValid);
    EXPECT_TRUE(OperationStep(0, OptoChannel::CH3, OpType::OFF).isValid);
    EXPECT_FALSE(OperationStep(30, static_cast<OptoChannel>(1), OpType::ON)
                     .isValid); // channel 1 reserved for IR LED
    EXPECT_FALSE(OperationStep(30, static_cast<OptoChannel>(4), OpType::ON)
                     .isValid); // out of range
    EXPECT_FALSE(OperationStep(30, OptoChannel::ALL, OpType::OFF)
                     .isValid); // ALL only for STOP

    // STOP is global: channel ALL (-1). Any non-negative frameIdx is allowed.
    EXPECT_TRUE(OperationStep(90, OptoChannel::ALL, OpType::STOP).isValid);
    EXPECT_TRUE(OperationStep(0, OptoChannel::ALL, OpType::STOP).isValid);
    EXPECT_TRUE(OperationStep(91, OptoChannel::ALL, OpType::STOP)
                    .isValid); // frameIdx need not be a multiple of anything
    EXPECT_FALSE(OperationStep(90, OptoChannel::CH2, OpType::STOP)
                     .isValid); // channel must be ALL

    // A default-constructed step is invalid (built from no fields).
    EXPECT_FALSE(OperationStep().isValid);
}

TEST(OperationStep, JsonRoundTrip) {
    OperationStep step(30, OptoChannel::CH2, OpType::ON);
    ASSERT_TRUE(step.isValid);

    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    step.toJson(obj);

    OperationStep parsed{JsonObjectConst(obj)};
    ASSERT_TRUE(parsed.isValid);
    EXPECT_EQ(parsed.frameIdx, step.frameIdx);
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
    EXPECT_FALSE(parsed.isValid);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

TEST(CommandBuilder, MakeStream) {
    Command cmd = Command::makeStreamCommand(validParams());
    EXPECT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::STREAM);
    expectParamsEqual(cmd.params, validParams());
}

TEST(CommandBuilder, MakeStopRecording) {
    Command cmd = Command::makeStopRecordingCommand();
    EXPECT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::STOP_RECORDING);
}

TEST(CommandBuilder, MakeLog) {
    Command cmd = Command::makeLogCommand("hello world");
    EXPECT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::LOG);
    EXPECT_EQ(cmd.logMsg, "hello world");
}

TEST(CommandBuilder, MakeStartRecordingValid) {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(OperationStep(60, OptoChannel::CH3, OpType::OFF));
    seq.push_back(OperationStep(90, OptoChannel::ALL, OpType::STOP));

    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), seq);
    EXPECT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::START_RECORDING);
    EXPECT_EQ(cmd.opSequence.size(), 3u);
}

TEST(CommandBuilder, MakeStartRecordingRejectsInvalidStep) {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(
        OperationStep(60, OptoChannel::ALL, OpType::ON)); // ALL invalid for ON

    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), seq);
    EXPECT_FALSE(cmd.isValid);
}

TEST(CommandBuilder, MakeStartRecordingEmptySequence) {
    // An open recording carries an empty opSequence and is still valid.
    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), {});
    EXPECT_TRUE(cmd.isValid);
    EXPECT_TRUE(cmd.opSequence.empty());
}

/* -------------------------------------------------------------------------- */
/* Serialization round trips                                                  */
/* -------------------------------------------------------------------------- */

TEST(RoundTrip, Stream) {
    Command original = Command::makeStreamCommand(validParams());
    std::string json = original.toString();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.isValid);
    EXPECT_EQ(parsed.cmdType, CmdType::STREAM);
    expectParamsEqual(parsed.params, original.params);
}

TEST(RoundTrip, StartRecording) {
    TriggerParams rec = validParams();
    TriggerParams revert = validParams();
    revert.behFrameRate = 50;     // distinguish the two param blocks
    revert.enableMuscle = true;   // rec/revert may carry different modes

    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(OperationStep(90, OptoChannel::ALL, OpType::STOP));

    Command original = Command::makeStartRecordingCommand(rec, revert, seq);
    std::string json = original.toString();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.isValid);
    EXPECT_EQ(parsed.cmdType, CmdType::START_RECORDING);
    expectParamsEqual(parsed.recParams, rec);
    expectParamsEqual(parsed.revertToParams, revert);
    EXPECT_NE(parsed.recParams.behFrameRate, parsed.revertToParams.behFrameRate);
    EXPECT_NE(parsed.recParams.enableMuscle, parsed.revertToParams.enableMuscle);

    ASSERT_EQ(parsed.opSequence.size(), 2u);
    EXPECT_EQ(parsed.opSequence[0].frameIdx, 30u);
    EXPECT_EQ(parsed.opSequence[0].channel, OptoChannel::CH2);
    EXPECT_EQ(parsed.opSequence[0].op, OpType::ON);
    EXPECT_EQ(parsed.opSequence[1].frameIdx, 90u);
    EXPECT_EQ(parsed.opSequence[1].channel, OptoChannel::ALL);
    EXPECT_EQ(parsed.opSequence[1].op, OpType::STOP);
}

TEST(RoundTrip, StopRecording) {
    Command original = Command::makeStopRecordingCommand();
    std::string json = original.toString();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.isValid);
    EXPECT_EQ(parsed.cmdType, CmdType::STOP_RECORDING);
}

TEST(RoundTrip, Log) {
    Command original = Command::makeLogCommand("a log line");
    std::string json = original.toString();
    ASSERT_FALSE(json.empty());

    Command parsed = Command::parse(json);
    ASSERT_TRUE(parsed.isValid);
    EXPECT_EQ(parsed.cmdType, CmdType::LOG);
    EXPECT_EQ(parsed.logMsg, "a log line");
}

/* -------------------------------------------------------------------------- */
/* Parsing failures                                                           */
/* -------------------------------------------------------------------------- */

TEST(Parse, MalformedJson) {
    Command cmd = Command::parse("{not valid json");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, UnknownCmdType) {
    Command cmd = Command::parse(R"({"cmdType":"NOPE"})");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, LegacyRunIsRejected) {
    // RUN was replaced by STREAM/START_RECORDING: it must no longer parse.
    Command cmd = Command::parse(R"({"cmdType":"RUN"})");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, MisspelledKeyIsRejected) {
    // The discriminator key is "cmdType", not the old "cmdTyp".
    Command cmd = Command::parse(R"({"cmdTyp":"LOG","msg":"x"})");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, LogMissingMsg) {
    Command cmd = Command::parse(R"({"cmdType":"LOG"})");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamMissingParams) {
    Command cmd = Command::parse(R"({"cmdType":"STREAM"})");
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamRejectsZeroFrameRate) {
    // behFrameRate must be strictly positive.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":true,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":0,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamRejectsMissingField) {
    // pcoCamReadoutTime is absent.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":true,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamRejectsMissingEnableMuscle) {
    // enableMuscle is a required field of every params block.
    std::string json =
        R"({"cmdType":"STREAM","params":{"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamRejectsNonBoolEnableMuscle) {
    // enableMuscle must be a JSON boolean, not a number or string.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":"yes","behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.isValid);
}

TEST(Parse, StreamAcceptsMuscleDisabled) {
    // A behavior-only params block (enableMuscle false) parses fine; the
    // muscle-only fields are still present but unused by the controller.
    std::string json =
        R"({"cmdType":"STREAM","params":{"enableMuscle":false,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    ASSERT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::STREAM);
    EXPECT_FALSE(cmd.params.enableMuscle);
}

TEST(Parse, StartRecordingMissingRevertParams) {
    std::string json =
        R"({"cmdType":"START_RECORDING","recParams":{"enableMuscle":true,)"
        R"("behExpTime":0,"muscEffExpTime":0,"behFrameRate":100,)"
        R"("behMuscSyncRatio":3,"pcoCamRollingTime":0,)"
        R"("pcoCamReadoutTime":0},"opSequence":[]})";
    Command cmd = Command::parse(json);
    EXPECT_FALSE(cmd.isValid);
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
    EXPECT_FALSE(cmd.isValid);
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
    EXPECT_FALSE(cmd.isValid);
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
    ASSERT_TRUE(cmd.isValid);
    EXPECT_EQ(cmd.cmdType, CmdType::START_RECORDING);
    EXPECT_TRUE(cmd.opSequence.empty());
}

} // namespace
