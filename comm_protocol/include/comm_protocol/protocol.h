#pragma once

#include <string>
#include <vector>

#include <ArduinoJson.h>

/**
 * Serial (USB) communication protocol between the recorder (host computer) and
 * the Arduino Nano ESP32 trigger controller, as specified in
 * docs/comm_protocol.md.
 *
 * Every message is a single compact JSON object discriminated by a top-level
 * "cmdTyp" field, which is one of the fixed literals "RUN" or "LOG":
 *
 *   RUN   - reconfigure the controller; carries "params" and "recording".
 *   LOG   - carries a free-form "msg" string to be echoed by the controller.
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
    RUN,
    LOG,
};

/** Optogenetics operation applied to a channel at a given frame. */
enum class OpType {
    ON,
    OFF,
    STOP,
};

enum class OptoChannel {
    ALL = -1,
    CH2 = 2,
    CH3 = 3,
};

/**
 * One entry of a RUN command's "recording/opSequence" list: after the
 * `frameIdx`-th behavior frame, apply operation `op` on optogenetics `channel`.
 *
 *   - frameIdx: non-negative frame index must be a multiple of 3 for STOP ops
 *   - channel: CCS channel; 2 or 3 when op is ON/OFF, -1 when op is STOP
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

/** Decoded "params" object of a RUN command (controller configuration). */
struct TriggerParams {
    bool pcoCamContinuous = false;       // PCO muscle cam continuous mode
    unsigned int behExpTime = 0;        // behavior cam exposure time (us)
    unsigned int muscEffExpTime = 0;    // muscle cam effective exposure (us)
    unsigned int behFrameRate = 0;      // behavior cam frame rate (fps), > 0
    unsigned int behMuscSyncRatio = 0;  // beh frames per muscle frame, > 0
    unsigned int pcoCamRollingTime = 0; // PCO sensor rolling time (us)
    unsigned int pcoCamReadoutTime = 0; // PCO total readout time (us)
};

/** Decoded "recording" object of a RUN command. */
struct Recording {
    bool isRecording = false;              // saving frames vs. live preview
    std::vector<OperationStep> opSequence; // empty => open recording
};

/**
 * A full protocol message.
 *
 * Build one with makeRunCommand()/makeLogCommand() and serialize with
 * toString(); decode an incoming line with parse(). `isValid` is false when a
 * message could not be parsed or is malformed, in which case the decoded fields
 * are meaningless.
 */
class Command {
  public:
    CmdType cmdType = CmdType::RUN;
    TriggerParams params;    // meaningful when cmdType == CmdType::RUN
    Recording recording; // meaningful when cmdType == CmdType::RUN
    std::string logMsg;  // meaningful when cmdType == CmdType::LOG
    bool isValid = false;

    Command() = default;

    /** Build a RUN command (validates every opSequence step). */
    static Command
    makeRunCommand(const TriggerParams &params, const Recording &recording);

    /** Build a LOG command carrying a free-form message. */
    static Command makeLogCommand(const std::string &message);

    /** Parse a single JSON message string; check isValid on the result. */
    static Command parse(const std::string &jsonStr);

    /** Serialize to a compact JSON string (empty string if !isValid). */
    std::string toString() const;
};
