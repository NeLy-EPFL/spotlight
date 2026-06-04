#include "comm_protocol/protocol.h"

#include <cstring>

namespace {

constexpr char kCmdStream[] = "STREAM";
constexpr char kCmdStartRecording[] = "START_RECORDING";
constexpr char kCmdStopRecording[] = "STOP_RECORDING";
constexpr char kCmdLog[] = "LOG";

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
 *   - ON/OFF act on a single optogenetics channel, which must be CH2 or CH3
 *     (channel 1 is reserved for the IR LED);
 *   - STOP is global, so its channel is ALL (-1).
 */
bool opStepIsValid(OptoChannel channel, OpType op) {
    switch (op) {
    case OpType::ON:
    case OpType::OFF:
        return channel == OptoChannel::CH2 || channel == OptoChannel::CH3;
    case OpType::STOP:
        return channel == OptoChannel::ALL;
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

/** Parse a "params"-shaped object into `out`; returns false on any bad field.
 */
bool parseParams(JsonObjectConst obj, TriggerParams &out) {
    if (obj.isNull()) {
        return false;
    }
    return getUint(obj, "behExpTime", out.behExpTime) &&
           getUint(obj, "muscEffExpTime", out.muscEffExpTime) &&
           getPositiveUint(obj, "behFrameRate", out.behFrameRate) &&
           getPositiveUint(obj, "behMuscSyncRatio", out.behMuscSyncRatio) &&
           getUint(obj, "pcoCamRollingTime", out.pcoCamRollingTime) &&
           getUint(obj, "pcoCamReadoutTime", out.pcoCamReadoutTime);
}

/** Write a "params"-shaped object from `params` into `obj`. */
void writeParams(JsonObject obj, const TriggerParams &params) {
    obj["behExpTime"] = params.behExpTime;
    obj["muscEffExpTime"] = params.muscEffExpTime;
    obj["behFrameRate"] = params.behFrameRate;
    obj["behMuscSyncRatio"] = params.behMuscSyncRatio;
    obj["pcoCamRollingTime"] = params.pcoCamRollingTime;
    obj["pcoCamReadoutTime"] = params.pcoCamReadoutTime;
}

} // namespace

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

OperationStep::OperationStep(
    unsigned long frameIdx, OptoChannel channel, OpType op)
    : frameIdx(frameIdx), channel(channel), op(op),
      isValid(opStepIsValid(channel, op)) {}

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
    isValid = opStepIsValid(channel, op);
}

void OperationStep::toJson(JsonObject obj) const {
    obj["frameIdx"] = frameIdx;
    obj["channel"] = static_cast<int>(channel);
    obj["op"] = opTypeToString(op);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

Command Command::makeStreamCommand(const TriggerParams &params) {
    Command cmd;
    cmd.cmdType = CmdType::STREAM;
    cmd.params = params;
    cmd.isValid = true;
    return cmd;
}

Command Command::makeStartRecordingCommand(
    const TriggerParams &recParams,
    const TriggerParams &revertToParams,
    const std::deque<OperationStep> &opSequence) {
    Command cmd;
    cmd.cmdType = CmdType::START_RECORDING;
    cmd.recParams = recParams;
    cmd.revertToParams = revertToParams;
    cmd.opSequence = opSequence;
    cmd.isValid = true;
    for (const OperationStep &step : opSequence) {
        if (!step.isValid) {
            cmd.isValid = false;
            break;
        }
    }
    return cmd;
}

Command Command::makeStopRecordingCommand() {
    Command cmd;
    cmd.cmdType = CmdType::STOP_RECORDING;
    cmd.isValid = true;
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

    JsonVariantConst cmdTypeVar = doc["cmdType"];
    if (!cmdTypeVar.is<const char *>()) {
        return cmd;
    }
    const char *cmdType = cmdTypeVar.as<const char *>();

    if (std::strcmp(cmdType, kCmdLog) == 0) {
        JsonVariantConst msg = doc["msg"];
        if (!msg.is<const char *>()) {
            return cmd;
        }
        cmd.cmdType = CmdType::LOG;
        cmd.logMsg = msg.as<const char *>();
        cmd.isValid = true;
        return cmd;
    }

    if (std::strcmp(cmdType, kCmdStopRecording) == 0) {
        cmd.cmdType = CmdType::STOP_RECORDING;
        cmd.isValid = true;
        return cmd;
    }

    if (std::strcmp(cmdType, kCmdStream) == 0) {
        if (!parseParams(doc["params"], cmd.params)) {
            return cmd;
        }
        cmd.cmdType = CmdType::STREAM;
        cmd.isValid = true;
        return cmd;
    }

    if (std::strcmp(cmdType, kCmdStartRecording) == 0) {
        if (!parseParams(doc["recParams"], cmd.recParams) ||
            !parseParams(doc["revertToParams"], cmd.revertToParams)) {
            return cmd;
        }

        JsonVariantConst seqVar = doc["opSequence"];
        if (!seqVar.is<JsonArrayConst>()) {
            return cmd; // required, but may be an empty array
        }
        for (JsonObjectConst stepObj : seqVar.as<JsonArrayConst>()) {
            OperationStep step(stepObj);
            if (!step.isValid) {
                return cmd;
            }
            cmd.opSequence.push_back(step);
        }

        cmd.cmdType = CmdType::START_RECORDING;
        cmd.isValid = true;
        return cmd;
    }

    return cmd; // unknown cmdType
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
    case CmdType::STREAM:
        doc["cmdType"] = kCmdStream;
        writeParams(doc["params"].to<JsonObject>(), params);
        break;
    case CmdType::START_RECORDING: {
        doc["cmdType"] = kCmdStartRecording;
        writeParams(doc["recParams"].to<JsonObject>(), recParams);
        writeParams(doc["revertToParams"].to<JsonObject>(), revertToParams);
        JsonArray seq = doc["opSequence"].to<JsonArray>();
        for (const OperationStep &step : opSequence) {
            // Defensive: never serialize an invalid step.
            if (!step.isValid) {
                return std::string();
            }
            step.toJson(seq.add<JsonObject>());
        }
        break;
    }
    case CmdType::STOP_RECORDING:
        doc["cmdType"] = kCmdStopRecording;
        break;
    case CmdType::LOG:
        doc["cmdType"] = kCmdLog;
        doc["msg"] = logMsg;
        break;
    }

    std::string out;
    serializeJson(doc, out);
    return out;
}
