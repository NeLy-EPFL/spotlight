// Unit tests for the comm_protocol message library.
//
// These tests are self-contained: they use a tiny assertion harness instead of
// an external framework so that the suite stays dependency-free and compiles on
// any toolchain that already builds comm_protocol. The single executable runs
// every test and returns a non-zero exit code if any check fails, which lets
// CTest report the overall result.

#include <comm_protocol/protocol.h>

#include <cstdio>
#include <string>

namespace {

int g_checks = 0;
int g_failures = 0;

void reportCheck(bool ok, const char *expr, const char *file, int line) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s:%d: %s\n", file, line, expr);
    }
}

} // namespace

#define CHECK(cond) reportCheck((cond), #cond, __FILE__, __LINE__)

namespace {

/** A TriggerParams instance with values that satisfy every field constraint. */
TriggerParams validParams() {
    TriggerParams p;
    p.behExpTime = 1000;
    p.muscEffExpTime = 2000;
    p.behFrameRate = 100;   // must be >= 1
    p.behMuscSyncRatio = 3; // must be >= 1
    p.pcoCamRollingTime = 50;
    p.pcoCamReadoutTime = 80;
    return p;
}

/** True if two TriggerParams have identical fields. */
bool paramsEqual(const TriggerParams &a, const TriggerParams &b) {
    return a.behExpTime == b.behExpTime &&
           a.muscEffExpTime == b.muscEffExpTime &&
           a.behFrameRate == b.behFrameRate &&
           a.behMuscSyncRatio == b.behMuscSyncRatio &&
           a.pcoCamRollingTime == b.pcoCamRollingTime &&
           a.pcoCamReadoutTime == b.pcoCamReadoutTime;
}

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

void testOperationStepValidity() {
    // ON/OFF act on channel CH2 or CH3.
    CHECK(OperationStep(30, OptoChannel::CH2, OpType::ON).isValid);
    CHECK(OperationStep(0, OptoChannel::CH3, OpType::OFF).isValid);
    CHECK(!OperationStep(30, static_cast<OptoChannel>(1), OpType::ON)
               .isValid); // channel 1 reserved for IR LED
    CHECK(!OperationStep(30, static_cast<OptoChannel>(4), OpType::ON)
               .isValid); // out of range
    CHECK(!OperationStep(30, OptoChannel::ALL, OpType::OFF)
               .isValid); // ALL only for STOP

    // STOP is global: channel ALL (-1). Any non-negative frameIdx is allowed.
    CHECK(OperationStep(90, OptoChannel::ALL, OpType::STOP).isValid);
    CHECK(OperationStep(0, OptoChannel::ALL, OpType::STOP).isValid);
    CHECK(OperationStep(91, OptoChannel::ALL, OpType::STOP)
              .isValid); // frameIdx need not be a multiple of anything
    CHECK(!OperationStep(90, OptoChannel::CH2, OpType::STOP)
               .isValid); // channel must be ALL

    // A default-constructed step is invalid (built from no fields).
    CHECK(!OperationStep().isValid);
}

void testOperationStepJsonRoundTrip() {
    OperationStep step(30, OptoChannel::CH2, OpType::ON);
    CHECK(step.isValid);

    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    step.toJson(obj);

    OperationStep parsed{JsonObjectConst(obj)};
    CHECK(parsed.isValid);
    CHECK(parsed.frameIdx == step.frameIdx);
    CHECK(parsed.channel == step.channel);
    CHECK(parsed.op == step.op);
}

void testOperationStepParseRejectsBadOp() {
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    obj["frameIdx"] = 30;
    obj["channel"] = 2;
    obj["op"] = "WIGGLE"; // not a known op

    OperationStep parsed{JsonObjectConst(obj)};
    CHECK(!parsed.isValid);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

void testMakeStream() {
    Command cmd = Command::makeStreamCommand(validParams());
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::STREAM);
    CHECK(paramsEqual(cmd.params, validParams()));
}

void testMakeStopRecording() {
    Command cmd = Command::makeStopRecordingCommand();
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::STOP_RECORDING);
}

void testMakeLog() {
    Command cmd = Command::makeLogCommand("hello world");
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::LOG);
    CHECK(cmd.logMsg == "hello world");
}

void testMakeStartRecordingValid() {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(OperationStep(60, OptoChannel::CH3, OpType::OFF));
    seq.push_back(OperationStep(90, OptoChannel::ALL, OpType::STOP));

    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), seq);
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::START_RECORDING);
    CHECK(cmd.opSequence.size() == 3);
}

void testMakeStartRecordingRejectsInvalidStep() {
    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(
        OperationStep(60, OptoChannel::ALL, OpType::ON)); // ALL invalid for ON

    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), seq);
    CHECK(!cmd.isValid);
}

void testMakeStartRecordingEmptySequence() {
    // An open recording carries an empty opSequence and is still valid.
    Command cmd =
        Command::makeStartRecordingCommand(validParams(), validParams(), {});
    CHECK(cmd.isValid);
    CHECK(cmd.opSequence.empty());
}

/* -------------------------------------------------------------------------- */
/* Serialization round trips                                                  */
/* -------------------------------------------------------------------------- */

void testStreamRoundTrip() {
    Command original = Command::makeStreamCommand(validParams());
    std::string json = original.toString();
    CHECK(!json.empty());

    Command parsed = Command::parse(json);
    CHECK(parsed.isValid);
    CHECK(parsed.cmdType == CmdType::STREAM);
    CHECK(paramsEqual(parsed.params, original.params));
}

void testStartRecordingRoundTrip() {
    TriggerParams rec = validParams();
    TriggerParams revert = validParams();
    revert.behFrameRate = 50; // distinguish the two param blocks

    std::deque<OperationStep> seq;
    seq.push_back(OperationStep(30, OptoChannel::CH2, OpType::ON));
    seq.push_back(OperationStep(90, OptoChannel::ALL, OpType::STOP));

    Command original = Command::makeStartRecordingCommand(rec, revert, seq);
    std::string json = original.toString();
    CHECK(!json.empty());

    Command parsed = Command::parse(json);
    CHECK(parsed.isValid);
    CHECK(parsed.cmdType == CmdType::START_RECORDING);
    CHECK(paramsEqual(parsed.recParams, rec));
    CHECK(paramsEqual(parsed.revertToParams, revert));
    CHECK(parsed.recParams.behFrameRate != parsed.revertToParams.behFrameRate);

    CHECK(parsed.opSequence.size() == 2);
    CHECK(parsed.opSequence[0].frameIdx == 30);
    CHECK(parsed.opSequence[0].channel == OptoChannel::CH2);
    CHECK(parsed.opSequence[0].op == OpType::ON);
    CHECK(parsed.opSequence[1].frameIdx == 90);
    CHECK(parsed.opSequence[1].channel == OptoChannel::ALL);
    CHECK(parsed.opSequence[1].op == OpType::STOP);
}

void testStopRecordingRoundTrip() {
    Command original = Command::makeStopRecordingCommand();
    std::string json = original.toString();
    CHECK(!json.empty());

    Command parsed = Command::parse(json);
    CHECK(parsed.isValid);
    CHECK(parsed.cmdType == CmdType::STOP_RECORDING);
}

void testLogRoundTrip() {
    Command original = Command::makeLogCommand("a log line");
    std::string json = original.toString();
    CHECK(!json.empty());

    Command parsed = Command::parse(json);
    CHECK(parsed.isValid);
    CHECK(parsed.cmdType == CmdType::LOG);
    CHECK(parsed.logMsg == "a log line");
}

/* -------------------------------------------------------------------------- */
/* Parsing failures                                                           */
/* -------------------------------------------------------------------------- */

void testParseMalformedJson() {
    Command cmd = Command::parse("{not valid json");
    CHECK(!cmd.isValid);
}

void testParseUnknownCmdType() {
    Command cmd = Command::parse(R"({"cmdType":"NOPE"})");
    CHECK(!cmd.isValid);
}

void testParseLegacyRunIsRejected() {
    // RUN was replaced by STREAM/START_RECORDING: it must no longer parse.
    Command cmd = Command::parse(R"({"cmdType":"RUN"})");
    CHECK(!cmd.isValid);
}

void testParseMisspelledKeyIsRejected() {
    // The discriminator key is "cmdType", not the old "cmdTyp".
    Command cmd = Command::parse(R"({"cmdTyp":"LOG","msg":"x"})");
    CHECK(!cmd.isValid);
}

void testParseLogMissingMsg() {
    Command cmd = Command::parse(R"({"cmdType":"LOG"})");
    CHECK(!cmd.isValid);
}

void testParseStreamMissingParams() {
    Command cmd = Command::parse(R"({"cmdType":"STREAM"})");
    CHECK(!cmd.isValid);
}

void testParseStreamRejectsZeroFrameRate() {
    // behFrameRate must be strictly positive.
    std::string json =
        R"({"cmdType":"STREAM","params":{"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":0,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0}})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseStreamRejectsMissingField() {
    // pcoCamReadoutTime is absent.
    std::string json =
        R"({"cmdType":"STREAM","params":{"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0}})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseStartRecordingMissingRevertParams() {
    std::string json =
        R"({"cmdType":"START_RECORDING","recParams":{"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0},"opSequence":[]})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseStartRecordingMissingOpSequence() {
    // opSequence is required even though it may be empty.
    std::string params =
        R"({"behExpTime":0,"muscEffExpTime":0,"behFrameRate":100,)"
        R"("behMuscSyncRatio":3,"pcoCamRollingTime":0,"pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params + "}";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseStartRecordingRejectsBadStep() {
    // A STOP step on a specific channel (rather than ALL) is invalid.
    std::string params =
        R"({"behExpTime":0,"muscEffExpTime":0,"behFrameRate":100,)"
        R"("behMuscSyncRatio":3,"pcoCamRollingTime":0,"pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params +
                       R"(,"opSequence":[{"frameIdx":90,"channel":2,)"
                       R"("op":"STOP"}]})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseStartRecordingAcceptsEmptyOpSequence() {
    std::string params =
        R"({"behExpTime":0,"muscEffExpTime":0,"behFrameRate":100,)"
        R"("behMuscSyncRatio":3,"pcoCamRollingTime":0,"pcoCamReadoutTime":0})";
    std::string json = R"({"cmdType":"START_RECORDING","recParams":)" + params +
                       R"(,"revertToParams":)" + params +
                       R"(,"opSequence":[]})";
    Command cmd = Command::parse(json);
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::START_RECORDING);
    CHECK(cmd.opSequence.empty());
}

} // namespace

int main() {
    testOperationStepValidity();
    testOperationStepJsonRoundTrip();
    testOperationStepParseRejectsBadOp();
    testMakeStream();
    testMakeStopRecording();
    testMakeLog();
    testMakeStartRecordingValid();
    testMakeStartRecordingRejectsInvalidStep();
    testMakeStartRecordingEmptySequence();
    testStreamRoundTrip();
    testStartRecordingRoundTrip();
    testStopRecordingRoundTrip();
    testLogRoundTrip();
    testParseMalformedJson();
    testParseUnknownCmdType();
    testParseLegacyRunIsRejected();
    testParseMisspelledKeyIsRejected();
    testParseLogMissingMsg();
    testParseStreamMissingParams();
    testParseStreamRejectsZeroFrameRate();
    testParseStreamRejectsMissingField();
    testParseStartRecordingMissingRevertParams();
    testParseStartRecordingMissingOpSequence();
    testParseStartRecordingRejectsBadStep();
    testParseStartRecordingAcceptsEmptyOpSequence();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
