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

/** A SetParams instance with values that satisfy every field constraint. */
SetParams validParams() {
    SetParams p;
    p.pcoCamContinuous = true;
    p.behExpTime = 1000;
    p.muscEffExpTime = 2000;
    p.behFrameRate = 100;   // must be >= 1
    p.behMuscSyncRatio = 3; // must be >= 1
    p.pcoCamRollingTime = 50;
    p.pcoCamReadoutTime = 80;
    return p;
}

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

void testOperationStepValidity() {
    // ON/OFF act on channel 2 or 3.
    CHECK(OperationStep(30, 2, OpType::ON).isValid);
    CHECK(OperationStep(0, 3, OpType::OFF).isValid);
    CHECK(!OperationStep(30, 1, OpType::ON).isValid);   // channel 1 reserved
    CHECK(!OperationStep(30, 4, OpType::ON).isValid);   // out of range
    CHECK(!OperationStep(30, -1, OpType::OFF).isValid); // -1 only for STOP

    // STOP is global: channel -1 and frameIdx a multiple of 3.
    CHECK(OperationStep(90, -1, OpType::STOP).isValid);
    CHECK(OperationStep(0, -1, OpType::STOP).isValid);
    CHECK(!OperationStep(91, -1, OpType::STOP).isValid); // not a multiple of 3
    CHECK(!OperationStep(90, 2, OpType::STOP).isValid);  // channel must be -1

    // A default-constructed step is invalid (built from no fields).
    CHECK(!OperationStep().isValid);
}

void testOperationStepJsonRoundTrip() {
    OperationStep step(30, 2, OpType::ON);
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

void testMakeLog() {
    Command cmd = Command::makeLogCommand("hello world");
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::LOG);
    CHECK(cmd.logMsg == "hello world");
}

void testMakeSetValid() {
    Recording rec;
    rec.isRecording = true;
    rec.opSequence.push_back(OperationStep(30, 2, OpType::ON));
    rec.opSequence.push_back(OperationStep(60, 3, OpType::OFF));
    rec.opSequence.push_back(OperationStep(90, -1, OpType::STOP));

    Command cmd = Command::makeSetCommand(validParams(), rec);
    CHECK(cmd.isValid);
    CHECK(cmd.cmdType == CmdType::SET);
    CHECK(cmd.recording.opSequence.size() == 3);
}

void testMakeSetRejectsInvalidStep() {
    Recording rec;
    rec.isRecording = true;
    rec.opSequence.push_back(OperationStep(30, 2, OpType::ON));
    rec.opSequence.push_back(
        OperationStep(91, -1, OpType::STOP)); // bad frameIdx

    Command cmd = Command::makeSetCommand(validParams(), rec);
    CHECK(!cmd.isValid);
}

void testMakeSetEmptySequence() {
    // An open recording carries an empty opSequence and is still valid.
    Recording rec;
    rec.isRecording = true;
    Command cmd = Command::makeSetCommand(validParams(), rec);
    CHECK(cmd.isValid);
    CHECK(cmd.recording.opSequence.empty());
}

/* -------------------------------------------------------------------------- */
/* Serialization round trips                                                  */
/* -------------------------------------------------------------------------- */

void testSetRoundTrip() {
    Recording rec;
    rec.isRecording = true;
    rec.opSequence.push_back(OperationStep(30, 2, OpType::ON));
    rec.opSequence.push_back(OperationStep(90, -1, OpType::STOP));

    Command original = Command::makeSetCommand(validParams(), rec);
    std::string json = original.toString();
    CHECK(!json.empty());

    Command parsed = Command::parse(json);
    CHECK(parsed.isValid);
    CHECK(parsed.cmdType == CmdType::SET);

    CHECK(parsed.params.pcoCamContinuous == original.params.pcoCamContinuous);
    CHECK(parsed.params.behExpTime == original.params.behExpTime);
    CHECK(parsed.params.muscEffExpTime == original.params.muscEffExpTime);
    CHECK(parsed.params.behFrameRate == original.params.behFrameRate);
    CHECK(parsed.params.behMuscSyncRatio == original.params.behMuscSyncRatio);
    CHECK(parsed.params.pcoCamRollingTime == original.params.pcoCamRollingTime);
    CHECK(parsed.params.pcoCamReadoutTime == original.params.pcoCamReadoutTime);

    CHECK(parsed.recording.isRecording == original.recording.isRecording);
    CHECK(parsed.recording.opSequence.size() == 2);
    CHECK(parsed.recording.opSequence[0].channel == 2);
    CHECK(parsed.recording.opSequence[0].op == OpType::ON);
    CHECK(parsed.recording.opSequence[1].frameIdx == 90);
    CHECK(parsed.recording.opSequence[1].op == OpType::STOP);
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
    Command cmd = Command::parse(R"({"cmdTyp":"NOPE"})");
    CHECK(!cmd.isValid);
}

void testParsePauseIsRejected() {
    // PAUSE was removed from the protocol: it must no longer parse.
    Command cmd = Command::parse(R"({"cmdTyp":"PAUSE"})");
    CHECK(!cmd.isValid);
}

void testParseLogMissingMsg() {
    Command cmd = Command::parse(R"({"cmdTyp":"LOG"})");
    CHECK(!cmd.isValid);
}

void testParseSetMissingParams() {
    Command cmd = Command::parse(
        R"({"cmdTyp":"SET","recording":{"isRecording":false,"opSequence":[]}})");
    CHECK(!cmd.isValid);
}

void testParseSetRejectsZeroFrameRate() {
    // behFrameRate must be strictly positive.
    std::string json =
        R"({"cmdTyp":"SET","params":{"pcoCamContinuous":false,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":0,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0},)"
        R"("recording":{"isRecording":false,"opSequence":[]}})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseSetRejectsBadStep() {
    // A STOP step with a frameIdx that is not a multiple of 3 is invalid.
    std::string json =
        R"({"cmdTyp":"SET","params":{"pcoCamContinuous":false,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0},)"
        R"("recording":{"isRecording":true,"opSequence":)"
        R"([{"frameIdx":91,"channel":-1,"op":"STOP"}]}})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

void testParseSetMissingOpSequence() {
    // opSequence is required even though it may be empty.
    std::string json =
        R"({"cmdTyp":"SET","params":{"pcoCamContinuous":false,"behExpTime":0,)"
        R"("muscEffExpTime":0,"behFrameRate":100,"behMuscSyncRatio":3,)"
        R"("pcoCamRollingTime":0,"pcoCamReadoutTime":0},)"
        R"("recording":{"isRecording":false}})";
    Command cmd = Command::parse(json);
    CHECK(!cmd.isValid);
}

} // namespace

int main() {
    testOperationStepValidity();
    testOperationStepJsonRoundTrip();
    testOperationStepParseRejectsBadOp();
    testMakeLog();
    testMakeSetValid();
    testMakeSetRejectsInvalidStep();
    testMakeSetEmptySequence();
    testSetRoundTrip();
    testLogRoundTrip();
    testParseMalformedJson();
    testParseUnknownCmdType();
    testParsePauseIsRejected();
    testParseLogMissingMsg();
    testParseSetMissingParams();
    testParseSetRejectsZeroFrameRate();
    testParseSetRejectsBadStep();
    testParseSetMissingOpSequence();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
