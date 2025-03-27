/**
 * @file parserAndFormatter.cpp
 * @brief Functions for parsing recording protocol strings.
 */

#include "parserAndFormatter.hpp"

/**
 * @brief Parses a single opto control step from a string and populates the
 * given ProtocolStep structure.
 * 
 * The input string should follow the format:
 * `<frameCount>/<channel>/<operation>`
 * For example:
 * `100/ch1/on`, `200/ch1/off`, `200/x/stop`
 * 
 * - `<frameCount>`: An integer representing the number of behavior frames
 *                   after which the event should happen.
 * - `<channel>`:    Either `ch<number>` or `x` (doesn't apply to a specific
 *                   channel; field ignored).
 * - `<operation>`:  One of the following:
 *                   - `on`:   Turns the channel on.
 *                   - `off`:  Turns the channel off.
 *                   - `stop`: Stops recording.
 * 
 * @param str  The input string to parse.
 * @param step A reference to an ProtocolStep structure to populate with
 *             parsed values.
 * @return The number of characters successfully parsed from the input string,
 *         or -1 if the format is invalid.
 * 
 * @note If the channel is specified as `x`, the `channel` field in the `step`
 *       structure will be set to -1.
 *       If the operation is `stop`, the `isDone` field in the `step` structure
 *       will be set to true.
 */
int parseSingleProtocolStep(const char* str, ProtocolStep& step) {
  const char* ptr = str;
  
  // Parse time
  char* endPtr;
  step.frameCount = strtol(ptr, &endPtr, 10);
  if (ptr == endPtr || *endPtr != '/') {
    return -1; // Invalid format
  }
  ptr = endPtr + 1; // Skip the '/'
  
  // Parse channel
  if (*ptr == 'x') {
    step.channel = -1; // command is not channel-specific; don't care
    ptr++; // Skip the 'x'
  } else {
    // Parse channel number
    if (strncmp(ptr, "ch", 2) == 0) {
      ptr += 2; // Skip "ch"
    } else {
      return -1; // Invalid format
    }
    step.channel = strtol(ptr, &endPtr, 10);
    if (ptr == endPtr) {
      return -1; // Invalid format
    }
    ptr = endPtr; // Update ptr
  }
  
  if (*ptr != '/') {
    return -1; // Invalid format
  }
  ptr++; // Skip the '/'
  
  // Parse operation
  if (strncmp(ptr, "on", 2) == 0) {
    step.operation = OPTO_ON;
    ptr += 2; // Skip "on"
  } else if (strncmp(ptr, "off", 3) == 0) {
    step.operation = OPTO_OFF;
    ptr += 3; // Skip "off"
  } else if (strncmp(ptr, "stop", 4) == 0) {
    step.isDone = true;
    ptr += 4; // Skip "stop"
  } else {
    return -1; // Invalid format
  }
  
  return ptr - str;
}

/**
 * @brief Parses an entire recording protocol string into an array of
 * ProtocolStep structures.
 *
 * This function takes a protocol string and parses it into individual steps,
 * storing them in the provided array of ProtocolStep structures. The
 * function stops parsing when the maximum number of steps is reached, the end
 * of the string is encountered, or an invalid format is detected.
 *
 * @param protocolStr A null-terminated string containing the recording protocol
 *                    protocol. Each step in the protocol should be separated
 *                    by a semicolon (';').
 * @param steps       An array of ProtocolStep structures where the parsed
 *                    steps will be stored.
 * @param maxSteps    The maximum number of steps that can be stored in the
 *                    steps array.
 * 
 * @return The number of steps parsed if the whole protocol is successfully
 *         parsed; returns 0 if the input is empty; returns -1 if the string
 *         is invalid.
 *
 * @note If the protocol string contains invalid formatting, parsing will stop
 *       at the first invalid step.
 */
int parseEntireProtocol(const char* protocolStr,
                        ProtocolStep* steps,
                        int maxSteps) {
  int stepCount = 0;
  
  // Check for null input
  if (!protocolStr || !steps || maxSteps <= 0) {
    return -1;
  }

  if (protocolStr[0] == ';' && protocolStr[1] == '\0') {
    return 0; // Empty string
  }
  
  const char* ptr = protocolStr;
  
  while (*ptr && stepCount < maxSteps) {
    ProtocolStep step;
    
    // Parse a single step
    int charsConsumed = parseSingleProtocolStep(ptr, step);
    if (charsConsumed < 0) {
      return -1; // Invalid format
    }
    
    // Add the step to the array
    steps[stepCount++] = step;
    
    // Move to the next step
    ptr += charsConsumed;
    
    // Skip to the next step
    if (*ptr == ';') {
      ptr++; // Skip the ';'
    } else if (*ptr == '\0') {
      break; // End of string
    } else {
      return -1;  // Invalid format
    }
  }
  return stepCount;
}

/**
 * @brief Formats a single protocol step into a string.
 * 
 * This function takes a ProtocolStep structure and converts it into a string
 * with the format `<frameCount>/<channel>/<operation>`.
 * 
 * @param step The ProtocolStep structure to format.
 * @return A string representing the formatted step.
 */
std::string formatProtocolStep(const ProtocolStep& step) {
    std::stringstream ss;
    ss << step.frameCount << "/";
    
    // Format channel
    if (step.channel == -1) {
        ss << "x";
    } else {
        ss << "ch" << step.channel;
    }
    
    ss << "/";
    
    // Format operation
    if (step.isDone) {
        ss << "stop";
    } else if (step.operation == OPTO_ON) {
        ss << "on";
    } else {
        ss << "off";
    }
    
    return ss.str();
}

/**
 * @brief Formats an entire protocol (vector of steps) into a string.
 * 
 * This function takes a vector of ProtocolStep structures and converts them
 * into a semicolon-separated string.
 * 
 * @param steps A vector of ProtocolStep structures.
 * @return A string representing the formatted protocol.
 */
std::string formatEntireProtocol(const std::vector<ProtocolStep>& steps) {
    if (steps.empty()) {
        return ";"; // Return empty protocol
    }
    
    std::stringstream ss;
    for (size_t i = 0; i < steps.size(); ++i) {
        ss << formatProtocolStep(steps[i]);
        if (i < steps.size() - 1) {
            ss << ";";
        }
    }
    return ss.str();
}

/**
 * @brief Formats a SET_BEHAVIOR_PERIOD command.
 * 
 * @param period The behavior camera period in microseconds.
 * @return A string containing the formatted command.
 */
std::string formatSetBehaviorPeriodCommand(unsigned int period) {
    std::stringstream ss;
    ss << CMDSTR_SET_BEHAVIOR_PERIOD << " " << period;
    return ss.str();
}

/**
 * @brief Formats a SET_SYNC_RATIO command.
 * 
 * @param ratio The sync ratio (k behavior triggers per muscle trigger).
 * @return A string containing the formatted command.
 */
std::string formatSetSyncRatioCommand(unsigned int ratio) {
    std::stringstream ss;
    ss << CMDSTR_SET_SYNC_RATIO << " " << ratio;
    return ss.str();
}

/**
 * @brief Formats a SET_BEHAVIOR_EXPOSURE_TIME command.
 * 
 * @param duration The behavior camera exposure time in microseconds.
 * @return A string containing the formatted command.
 */
std::string formatSetBehaviorExposureTimeCommand(unsigned int exposureTimeUs) {
    std::stringstream ss;
    ss << CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME << " " << exposureTimeUs;
    return ss.str();
}

/**
 * @brief Formats a SET_MUSCLE_EXPOSURE_TIME command.
 * 
 * @param duration The muscle camera exposure time in microseconds.
 * @return A string containing the formatted command.
 */
std::string formatSetMuscleExposureTimeCommand(unsigned int exposureTimeUs) {
    std::stringstream ss;
    ss << CMDSTR_SET_MUSCLE_EXPOSURE_TIME << " " << exposureTimeUs;
    return ss.str();
}

/**
 * @brief Formats a START_TRIGGERING command with an optional protocol.
 * 
 * @param protocol The protocol string to include, or an empty string.
 * @return A string containing the formatted command.
 */
std::string formatStartTriggeringCommand(const std::string& protocol) {
    std::stringstream ss;
    ss << CMDSTR_START_TRIGGERING;
    if (!protocol.empty()) {
        ss << " " << protocol;
    }
    return ss.str();
}

/**
 * @brief Formats a STOP_TRIGGERING command.
 * 
 * @return A string containing the formatted command.
 */
std::string formatStopTriggeringCommand() {
    return CMDSTR_STOP_TRIGGERING;
}