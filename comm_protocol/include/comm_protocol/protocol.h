#pragma once

#include <deque>
#include <string>

#include <ArduinoJson.h>

/**
 * Serial (USB) communication protocol between the recorder (host computer) and
 * the Arduino Nano ESP32 trigger controller, as specified in
 * docs/comm_protocol.md.
 *
 * Every message is a single compact JSON object discriminated by a top-level
 * "cmdType" field, which is one of the fixed literals "STREAM",
 * "START_RECORDING", "STOP_RECORDING", or "LOG":
 *
 *   STREAM          - live preview; carries "params".
 *   START_RECORDING - begin recording; carries "recParams", "revertToParams",
 *                     and an "opSequence" array (possibly empty).
 *   STOP_RECORDING  - end an open recording; carries no payload.
 *   LOG             - carries a free-form "msg" string to be echoed.
 *
 * The classes below parse and serialize these messages. They are meant to
 * compile and run unchanged on both a desktop computer and the ESP32:
 *   - no exceptions are used for control flow; parsing and construction report
 *     failure through the `isValid` flag (so a bad message never unwinds the
 *     stack on the MCU),
 *   - ArduinoJson handles the low-level tokenizing and number formatting.
 */

/** The command kinds carried over the serial link. */
enum class CmdType {
    STREAM,
    START_RECORDING,
    STOP_RECORDING,
    LOG,
};

/** Optogenetics operation applied to a channel at a given frame. */
enum class OpType {
    ON,
    OFF,
    STOP,
};

/**
 * Optogenetics channel. CH2 and CH3 are the controllable channels (channel 1
 * is reserved for the IR LED). ALL (-1) denotes a global operation and is used
 * with STOP, which reverts the controller to streaming.
 */
enum class OptoChannel {
    ALL = -1,
    CH2 = 2,
    CH3 = 3,
};

/**
 * One entry of a START_RECORDING command's "opSequence" list: after the
 * `frameIdx`-th behavior frame, apply operation `op` on optogenetics `channel`.
 *
 *   - frameIdx: non-negative behavior-frame index
 *   - channel: CH2 or CH3 when op is ON/OFF; ALL (-1) when op is STOP
 *   - op: ON, OFF, or STOP
 *
 * `isValid` is false when the step was built from fields that violate the rules
 * above or parsed from a malformed JSON object.
 */
class OperationStep {
  public:
    unsigned long frameIdx = 0;
    OptoChannel channel = OptoChannel::ALL;
    OpType op = OpType::STOP;
    bool isValid = false;

    OperationStep() = default;
    OperationStep(unsigned long frameIdx, OptoChannel channel, OpType op);

    /** Parse from one JSON object of the opSequence array. */
    explicit OperationStep(JsonObjectConst obj);

    /** Write this step into an (empty) JSON object. */
    void toJson(JsonObject obj) const;
};

/**
 * Decoded "params" object (controller configuration). The same shape is used
 * for STREAM's "params" and START_RECORDING's "recParams"/"revertToParams".
 */
struct TriggerParams {
    // When true, the controller locks behavior acquisition to the free-running
    // muscle (PCO) camera's common-time signal and pulses the blue excitation
    // LED. When false, the muscle camera is ignored entirely: the controller
    // free-runs the behavior camera on its own clock at behFrameRate, never
    // pulses the blue LED, and the muscle-only fields below (muscEffExpTime,
    // behMuscSyncRatio, pcoCamRollingTime, pcoCamReadoutTime) are unused.
    bool enableMuscle = true;
    unsigned int behExpTime = 0;        // behavior cam exposure time (us)
    unsigned int muscEffExpTime = 0;    // muscle cam effective exposure (us)
    unsigned int behFrameRate = 1;      // behavior cam frame rate (fps), > 0
    unsigned int behMuscSyncRatio = 1;  // beh frames per muscle frame, > 0
    unsigned int pcoCamRollingTime = 0; // PCO sensor rolling time (us)
    unsigned int pcoCamReadoutTime = 0; // PCO total readout time (us)
};

/**
 * A full protocol message.
 *
 * Build one with the make*Command() factories and serialize with toString();
 * decode an incoming line with parse(). `isValid` is false when a message could
 * not be parsed or is malformed, in which case the decoded fields are
 * meaningless. Which fields are meaningful depends on `cmdType`:
 *
 *   STREAM          -> params
 *   START_RECORDING -> recParams, revertToParams, opSequence
 *   STOP_RECORDING  -> (none)
 *   LOG             -> logMsg
 */
class Command {
  public:
    CmdType cmdType = CmdType::STREAM;

    TriggerParams params;         // STREAM
    TriggerParams recParams;      // START_RECORDING: params while recording
    TriggerParams revertToParams; // START_RECORDING: params after recording
    std::deque<OperationStep> opSequence; // START_RECORDING: empty => open
    std::string logMsg;                   // LOG

    bool isValid = false;

    Command() = default;

    /** Build a STREAM command. */
    static Command makeStreamCommand(const TriggerParams &params);

    /** Build a START_RECORDING command (validates every opSequence step). */
    static Command makeStartRecordingCommand(
        const TriggerParams &recParams,
        const TriggerParams &revertToParams,
        const std::deque<OperationStep> &opSequence);

    /** Build a STOP_RECORDING command. */
    static Command makeStopRecordingCommand();

    /** Build a LOG command carrying a free-form message. */
    static Command makeLogCommand(const std::string &message);

    /** Parse a single JSON message string; check isValid on the result. */
    static Command parse(const std::string &jsonStr);

    /** Serialize to a compact JSON string (empty string if !isValid). */
    std::string toString() const;
};

/**
 * Time to wait after receiving a START_RECORDING command before the trigger
 * logic starts. This allows pending frames in the camera buffers to be flushed
 * out, so the recorded session doesn't start with frames acquired using stale
 * parameters.
 *
 * On the recorder side, the program should ignore frames received **during a
 * fraction (e.g., 0.8x) of this time** after sending the command. If the ignore
 * period is too long, the first few frames of the recording will be incorrectly
 * dropped.
 */
inline constexpr unsigned long camFlushTimeUs = 100000;
