/**
 * @file parserAndFormatter.hpp
 * @brief Functions for parsing and formatting protocol strings.
 */

#ifndef PARSER_AND_FORMATTER_HPP
#define PARSER_AND_FORMATTER_HPP

#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <cstring>

const char CMDSTR_SET_BEHAVIOR_PERIOD[] = "SET_BEHAVIOR_PERIOD";
const char CMDSTR_SET_SYNC_RATIO[] = "SET_SYNC_RATIO";
const char CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME[] = "SET_BEHAVIOR_EXPOSURE_TIME";
const char CMDSTR_SET_MUSCLE_EXPOSURE_TIME[] = "SET_MUSCLE_EXPOSURE_TIME";
const char CMDSTR_START_TRIGGERING[] = "START_TRIGGERING";
const char CMDSTR_STOP_TRIGGERING[] = "STOP_TRIGGERING";
const char ACK_SUFFIX[] = "_ACK";
const size_t CMDLEN_SET_BEHAVIOR_PERIOD = sizeof(CMDSTR_SET_BEHAVIOR_PERIOD) - 1;
const size_t CMDLEN_SET_SYNC_RATIO = sizeof(CMDSTR_SET_SYNC_RATIO) - 1;
const size_t CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME = sizeof(CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME) - 1;
const size_t CMDLEN_SET_MUSCLE_EXPOSURE_TIME = sizeof(CMDSTR_SET_MUSCLE_EXPOSURE_TIME) - 1;
const size_t CMDLEN_START_TRIGGERING = sizeof(CMDSTR_START_TRIGGERING) - 1;
const size_t CMDLEN_STOP_TRIGGERING = sizeof(CMDSTR_STOP_TRIGGERING) - 1;

// Enums and structs matching those in the Arduino code
enum OptoOperation {
    OPTO_OFF,
    OPTO_ON
};

struct ProtocolStep {
    long frameCount = -1;
    int channel = -1;
    OptoOperation operation = OPTO_OFF;
    bool isDone = false;
};

// Parser functions (from existing parser.cpp)
int parseSingleProtocolStep(const char* str, ProtocolStep& step);
int parseEntireProtocol(const char* protocolStr, ProtocolStep* steps, int maxSteps);

// New formatter functions
std::string formatProtocolStep(const ProtocolStep& step);
std::string formatEntireProtocol(const std::vector<ProtocolStep>& steps);

// Command formatting functions
std::string formatSetBehaviorPeriodCommand(unsigned int period);
std::string formatSetSyncRatioCommand(unsigned int ratio);
std::string formatSetBehaviorExposureTimeCommand(unsigned int exposureTimeUs);
std::string formatSetMuscleExposureTimeCommand(unsigned int exposureTimeUs);
std::string formatStartTriggeringCommand(const std::string& protocol = "");
std::string formatStopTriggeringCommand();

#endif // PARSER_AND_FORMATTER_HPP