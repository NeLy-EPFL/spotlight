#include "comm_protocol/protocol.h"

#include <cstring>

namespace {

constexpr char cmd_stream[] = "STREAM";
constexpr char cmd_start_recording[] = "START_RECORDING";
constexpr char cmd_stop_recording[] = "STOP_RECORDING";
constexpr char cmd_log[] = "LOG";
constexpr char cmd_reset[] = "RESET";

constexpr char op_on[] = "ON";
constexpr char op_off[] = "OFF";
constexpr char op_stop[] = "STOP";

const char *op_type_to_string(OpType op) {
    switch (op) {
    case OpType::on:
        return op_on;
    case OpType::off:
        return op_off;
    case OpType::stop:
    default:
        return op_stop;
    }
}

bool string_to_op_type(const char *s, OpType &out) {
    if (std::strcmp(s, op_on) == 0) {
        out = OpType::on;
        return true;
    }
    if (std::strcmp(s, op_off) == 0) {
        out = OpType::off;
        return true;
    }
    if (std::strcmp(s, op_stop) == 0) {
        out = OpType::stop;
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
bool op_step_is_valid(OptoChannel channel, OpType op) {
    switch (op) {
    case OpType::on:
    case OpType::off:
        return channel == OptoChannel::ch2 || channel == OptoChannel::ch3;
    case OpType::stop:
        return channel == OptoChannel::all;
    }
    return false;
}

/** Read a required boolean field. */
bool get_bool(JsonObjectConst obj, const char *key, bool &out) {
    JsonVariantConst v = obj[key];
    if (!v.is<bool>()) {
        return false;
    }
    out = v.as<bool>();
    return true;
}

/** Read a required non-negative integer field. */
bool get_uint(JsonObjectConst obj, const char *key, unsigned int &out) {
    JsonVariantConst v = obj[key];
    if (!v.is<unsigned int>()) {
        return false;
    }
    out = v.as<unsigned int>();
    return true;
}

/** Read a required strictly-positive integer field. */
bool get_positive_uint(
    JsonObjectConst obj, const char *key, unsigned int &out) {
    return get_uint(obj, key, out) && out >= 1;
}

/** Parse a "params"-shaped object into `out`; returns false on any bad field.
 */
bool parse_params(JsonObjectConst obj, TriggerParams &out) {
    if (obj.isNull()) {
        return false;
    }
    return get_bool(obj, "enableMuscle", out.enable_muscle) &&
           get_uint(obj, "behExpTime", out.beh_exp_time) &&
           get_uint(obj, "muscEffExpTime", out.musc_eff_exp_time) &&
           get_positive_uint(obj, "behFrameRate", out.beh_frame_rate) &&
           get_positive_uint(
               obj, "behMuscSyncRatio", out.beh_musc_sync_ratio) &&
           get_uint(obj, "pcoCamRollingTime", out.pco_cam_rolling_time) &&
           get_uint(obj, "pcoCamReadoutTime", out.pco_cam_readout_time);
}

/** Write a "params"-shaped object from `params` into `obj`. */
void write_params(JsonObject obj, const TriggerParams &params) {
    obj["enableMuscle"] = params.enable_muscle;
    obj["behExpTime"] = params.beh_exp_time;
    obj["muscEffExpTime"] = params.musc_eff_exp_time;
    obj["behFrameRate"] = params.beh_frame_rate;
    obj["behMuscSyncRatio"] = params.beh_musc_sync_ratio;
    obj["pcoCamRollingTime"] = params.pco_cam_rolling_time;
    obj["pcoCamReadoutTime"] = params.pco_cam_readout_time;
}

} // namespace

/* -------------------------------------------------------------------------- */
/* OperationStep                                                              */
/* -------------------------------------------------------------------------- */

OperationStep::OperationStep(
    unsigned long frame_idx, OptoChannel channel, OpType op)
    : frame_idx(frame_idx), channel(channel), op(op),
      is_valid(op_step_is_valid(channel, op)) {}

OperationStep::OperationStep(JsonObjectConst obj) {
    JsonVariantConst frame_var = obj["frameIdx"];
    JsonVariantConst channel_var = obj["channel"];
    JsonVariantConst op_var = obj["op"];

    if (!frame_var.is<unsigned long>() || !channel_var.is<int>() ||
        !op_var.is<const char *>()) {
        return;
    }
    if (!string_to_op_type(op_var.as<const char *>(), op)) {
        return;
    }

    frame_idx = frame_var.as<unsigned long>();
    channel = static_cast<OptoChannel>(channel_var.as<int>());
    is_valid = op_step_is_valid(channel, op);
}

void OperationStep::to_json(JsonObject obj) const {
    obj["frameIdx"] = frame_idx;
    obj["channel"] = static_cast<int>(channel);
    obj["op"] = op_type_to_string(op);
}

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

Command Command::make_stream_command(const TriggerParams &params) {
    Command cmd;
    cmd.cmd_type = CmdType::stream;
    cmd.params = params;
    cmd.is_valid = true;
    return cmd;
}

Command Command::make_start_recording_command(
    const TriggerParams &rec_params,
    const TriggerParams &revert_to_params,
    const std::deque<OperationStep> &op_sequence) {
    Command cmd;
    cmd.cmd_type = CmdType::start_recording;
    cmd.rec_params = rec_params;
    cmd.revert_to_params = revert_to_params;
    cmd.op_sequence = op_sequence;
    cmd.is_valid = true;
    for (const OperationStep &step : op_sequence) {
        if (!step.is_valid) {
            cmd.is_valid = false;
            break;
        }
    }
    return cmd;
}

Command Command::make_stop_recording_command() {
    Command cmd;
    cmd.cmd_type = CmdType::stop_recording;
    cmd.is_valid = true;
    return cmd;
}

Command Command::make_log_command(const std::string &message) {
    Command cmd;
    cmd.cmd_type = CmdType::log;
    cmd.log_msg = message;
    cmd.is_valid = true;
    return cmd;
}

Command Command::make_reset_command() {
    Command cmd;
    cmd.cmd_type = CmdType::reset;
    cmd.is_valid = true;
    return cmd;
}

/* -------------------------------------------------------------------------- */
/* Parsing                                                                    */
/* -------------------------------------------------------------------------- */

Command Command::parse(const std::string &json_str) {
    Command cmd;

    JsonDocument doc;
    if (deserializeJson(doc, json_str)) {
        return cmd; // malformed JSON
    }

    JsonVariantConst cmd_type_var = doc["cmdType"];
    if (!cmd_type_var.is<const char *>()) {
        return cmd;
    }
    const char *cmd_type = cmd_type_var.as<const char *>();

    if (std::strcmp(cmd_type, cmd_log) == 0) {
        JsonVariantConst msg = doc["msg"];
        if (!msg.is<const char *>()) {
            return cmd;
        }
        cmd.cmd_type = CmdType::log;
        cmd.log_msg = msg.as<const char *>();
        cmd.is_valid = true;
        return cmd;
    }

    if (std::strcmp(cmd_type, cmd_stop_recording) == 0) {
        cmd.cmd_type = CmdType::stop_recording;
        cmd.is_valid = true;
        return cmd;
    }

    if (std::strcmp(cmd_type, cmd_reset) == 0) {
        cmd.cmd_type = CmdType::reset;
        cmd.is_valid = true;
        return cmd;
    }

    if (std::strcmp(cmd_type, cmd_stream) == 0) {
        if (!parse_params(doc["params"], cmd.params)) {
            return cmd;
        }
        cmd.cmd_type = CmdType::stream;
        cmd.is_valid = true;
        return cmd;
    }

    if (std::strcmp(cmd_type, cmd_start_recording) == 0) {
        if (!parse_params(doc["recParams"], cmd.rec_params) ||
            !parse_params(doc["revertToParams"], cmd.revert_to_params)) {
            return cmd;
        }

        JsonVariantConst seq_var = doc["opSequence"];
        if (!seq_var.is<JsonArrayConst>()) {
            return cmd; // required, but may be an empty array
        }
        for (JsonObjectConst step_obj : seq_var.as<JsonArrayConst>()) {
            OperationStep step(step_obj);
            if (!step.is_valid) {
                return cmd;
            }
            cmd.op_sequence.push_back(step);
        }

        cmd.cmd_type = CmdType::start_recording;
        cmd.is_valid = true;
        return cmd;
    }

    return cmd; // unknown cmdType
}

/* -------------------------------------------------------------------------- */
/* Serialization                                                              */
/* -------------------------------------------------------------------------- */

std::string Command::to_string() const {
    if (!is_valid) {
        return std::string();
    }

    JsonDocument doc;

    switch (cmd_type) {
    case CmdType::stream:
        doc["cmdType"] = cmd_stream;
        write_params(doc["params"].to<JsonObject>(), params);
        break;
    case CmdType::start_recording: {
        doc["cmdType"] = cmd_start_recording;
        write_params(doc["recParams"].to<JsonObject>(), rec_params);
        write_params(doc["revertToParams"].to<JsonObject>(), revert_to_params);
        JsonArray seq = doc["opSequence"].to<JsonArray>();
        for (const OperationStep &step : op_sequence) {
            // Defensive: never serialize an invalid step.
            if (!step.is_valid) {
                return std::string();
            }
            step.to_json(seq.add<JsonObject>());
        }
        break;
    }
    case CmdType::stop_recording:
        doc["cmdType"] = cmd_stop_recording;
        break;
    case CmdType::log:
        doc["cmdType"] = cmd_log;
        doc["msg"] = log_msg;
        break;
    case CmdType::reset:
        doc["cmdType"] = cmd_reset;
        break;
    }

    std::string out;
    serializeJson(doc, out);
    return out;
}
