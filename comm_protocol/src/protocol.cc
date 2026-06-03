#include "comm_protocol/protocol.h"

#include <cstring>
#include <utility>

namespace {

constexpr char kCmdTypRun[] = "RUN";
constexpr char kCmdTypLog[] = "LOG";

constexpr char kOpOn[] = "ON";
constexpr char kOpOff[] = "OFF";
constexpr char kOpStop[] = "STOP";

const char *opTypeToString(OpType op) {
    switch (op) {
    case OpType::ON:
        return kOpOn;
    case OpType::OFF:
        return kOpOff;
    case OpType::STOP:
    default:
        return kOpStop;
    }
}

bool stringToOpType(const char *s, OpType &out) {
    if (std::strcmp(s, kOpOn) == 0) {
        out = OpType::ON;
        return true;
    }
    if (std::strcmp(s, kOpOff) == 0) {
        out = OpType::OFF;
        return true;
    }
    if (std::strcmp(s, kOpStop) == 0) {
        out = OpType::STOP;
        return true;
    }
    return false;
}

/**
 * Operation-specific validity for one operation step:
 *   - ON/OFF act on a single optogenetics channel, which must be 2 or 3 (1 is
 *     reserved for the IR LED);
 *   - STOP is global, so its channel is -1, and its frameIdx must be a multiple
 *     of 3 (a hardware constraint enforced on both serialization and parsing).
 */
bool opStepIsValid(unsigned long frameIdx, OptoChannel channel, OpType op) {
    switch (op) {
    case OpType::ON:
    case OpType::OFF:
        return channel == OptoChannel::CH2 || channel == OptoChannel::CH3;
    case OpType::STOP:
        return channel == OptoChannel::ALL && (frameIdx % 3 == 0);
    }
    return false;
}

/** Read a required non-negative integer field. */
bool getUint(JsonObjectConst obj, const char *key, unsigned int &out) {
    JsonVariantConst v = obj[key];
    if (!v.is<unsigned int>()) {
        return false;
    }
    out = v.as<unsigned int>();
    return true;
}

/** Read a required strictly-positive integer field. */
bool getPositiveUint(JsonObjectConst obj, const char *key, unsigned int &out) {
    return getUint(obj, key, out) && out >= 1;
}

/** Read a required boolean field. */
bool getBool(JsonObjectConst obj, const char *key, bool &out) {
    JsonVariantConst v = obj[key];
    if (!v.is<bool>()) {
        return false;
    }
    out = v.as<bool>();
    return true;
}

} // namespace

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

OperationStep::OperationStep(
    unsigned long frameIdx, OptoChannel channel, OpType op)
    : frameIdx(frameIdx), channel(channel), op(op),
      isValid(opStepIsValid(frameIdx, channel, op)) {}

OperationStep::OperationStep(JsonObjectConst obj) {
    JsonVariantConst frameVar = obj["frameIdx"];
    JsonVariantConst channelVar = obj["channel"];
    JsonVariantConst opVar = obj["op"];

    if (!frameVar.is<unsigned long>() || !channelVar.is<int>() ||
        !opVar.is<const char *>()) {
        return;
    }
    if (!stringToOpType(opVar.as<const char *>(), op)) {
        return;
    }

    frameIdx = frameVar.as<unsigned long>();
    channel = static_cast<OptoChannel>(channelVar.as<int>());
    isValid = opStepIsValid(frameIdx, channel, op);
}

void OperationStep::toJson(JsonObject obj) const {
    obj["frameIdx"] = frameIdx;
    obj["channel"] = static_cast<int>(channel);
    obj["op"] = opTypeToString(op);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

Command Command::makeRunCommand(
    const TriggerParams &params, const Recording &recording) {
    Command cmd;
    cmd.cmdType = CmdType::RUN;
    cmd.params = params;
    cmd.recording = recording;
    cmd.isValid = true;
    for (const OperationStep &step : recording.opSequence) {
        if (!step.isValid) {
            cmd.isValid = false;
            break;
        }
    }
    return cmd;
}

Command Command::makeLogCommand(const std::string &message) {
    Command cmd;
    cmd.cmdType = CmdType::LOG;
    cmd.logMsg = message;
    cmd.isValid = true;
    return cmd;
}

/* -------------------------------------------------------------------------- */
/* Parsing                                                                    */
/* -------------------------------------------------------------------------- */

Command Command::parse(const std::string &jsonStr) {
    Command cmd;

    JsonDocument doc;
    if (deserializeJson(doc, jsonStr)) {
        return cmd; // malformed JSON
    }

    JsonVariantConst cmdTypVar = doc["cmdTyp"];
    if (!cmdTypVar.is<const char *>()) {
        return cmd;
    }
    const char *cmdTyp = cmdTypVar.as<const char *>();

    if (std::strcmp(cmdTyp, kCmdTypLog) == 0) {
        JsonVariantConst msg = doc["msg"];
        if (!msg.is<const char *>()) {
            return cmd;
        }
        cmd.cmdType = CmdType::LOG;
        cmd.logMsg = msg.as<const char *>();
        cmd.isValid = true;
        return cmd;
    }

    if (std::strcmp(cmdTyp, kCmdTypRun) != 0) {
        return cmd; // unknown cmdTyp
    }

    JsonObjectConst paramsObj = doc["params"];
    if (paramsObj.isNull()) {
        return cmd;
    }

    TriggerParams params;
    bool paramsOk =
        getBool(paramsObj, "pcoCamContinuous", params.pcoCamContinuous) &&
        getUint(paramsObj, "behExpTime", params.behExpTime) &&
        getUint(paramsObj, "muscEffExpTime", params.muscEffExpTime) &&
        getPositiveUint(paramsObj, "behFrameRate", params.behFrameRate) &&
        getPositiveUint(
            paramsObj, "behMuscSyncRatio", params.behMuscSyncRatio) &&
        getUint(paramsObj, "pcoCamRollingTime", params.pcoCamRollingTime) &&
        getUint(paramsObj, "pcoCamReadoutTime", params.pcoCamReadoutTime);
    if (!paramsOk) {
        return cmd;
    }

    JsonObjectConst recordingObj = doc["recording"];
    if (recordingObj.isNull()) {
        return cmd;
    }

    Recording recording;
    if (!getBool(recordingObj, "isRecording", recording.isRecording)) {
        return cmd;
    }

    JsonVariantConst seqVar = recordingObj["opSequence"];
    if (!seqVar.is<JsonArrayConst>()) {
        return cmd; // required, but may be an empty array
    }
    for (JsonObjectConst stepObj : seqVar.as<JsonArrayConst>()) {
        OperationStep step(stepObj);
        if (!step.isValid) {
            return cmd;
        }
        recording.opSequence.push_back(step);
    }

    cmd.cmdType = CmdType::RUN;
    cmd.params = params;
    cmd.recording = std::move(recording);
    cmd.isValid = true;
    return cmd;
}

/* -------------------------------------------------------------------------- */
/* Serialization                                                              */
/* -------------------------------------------------------------------------- */

std::string Command::toString() const {
    if (!isValid) {
        return std::string();
    }

    JsonDocument doc;

    switch (cmdType) {
    case CmdType::LOG:
        doc["cmdTyp"] = kCmdTypLog;
        doc["msg"] = logMsg;
        break;
    case CmdType::RUN: {
        doc["cmdTyp"] = kCmdTypRun;

        JsonObject paramsObj = doc["params"].to<JsonObject>();
        paramsObj["pcoCamContinuous"] = params.pcoCamContinuous;
        paramsObj["behExpTime"] = params.behExpTime;
        paramsObj["muscEffExpTime"] = params.muscEffExpTime;
        paramsObj["behFrameRate"] = params.behFrameRate;
        paramsObj["behMuscSyncRatio"] = params.behMuscSyncRatio;
        paramsObj["pcoCamRollingTime"] = params.pcoCamRollingTime;
        paramsObj["pcoCamReadoutTime"] = params.pcoCamReadoutTime;

        JsonObject recordingObj = doc["recording"].to<JsonObject>();
        recordingObj["isRecording"] = recording.isRecording;
        JsonArray seq = recordingObj["opSequence"].to<JsonArray>();
        for (const OperationStep &step : recording.opSequence) {
            // Defensive: never serialize an invalid step.
            if (!step.isValid) {
                return std::string();
            }
            JsonObject obj = seq.add<JsonObject>();
            step.toJson(obj);
        }
        break;
    }
    }

    std::string out;
    serializeJson(doc, out);
    return out;
}
