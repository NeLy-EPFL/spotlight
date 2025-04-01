#ifndef EXPERIMENT_PROTOCOL_HPP
#define EXPERIMENT_PROTOCOL_HPP

#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <sstream>
#include <optional>

inline const char *CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME = ">SET_BEHAVIOR_EXPOSURE_TIME";
inline const char *CMDSTR_SET_MUSCLE_EXPOSURE_TIME = ">SET_MUSCLE_EXPOSURE_TIME";
inline const char *CMDSTR_SET_BEHAVIOR_FPS = ">SET_BEHAVIOR_FPS";
inline const char *CMDSTR_SET_SYNC_RATIO = ">SET_SYNC_RATIO";
inline const char *CMDSTR_START_RECORDING = ">START_RECORDING";
inline const char *CMDSTR_STOP_RECORDING = ">STOP_RECORDING";

const int CMDLEN_SET_BEHAVIOR_EXPOSURE_TIME = strlen(CMDSTR_SET_BEHAVIOR_EXPOSURE_TIME);
const int CMDLEN_SET_MUSCLE_EXPOSURE_TIME = strlen(CMDSTR_SET_MUSCLE_EXPOSURE_TIME);
const int CMDLEN_SET_BEHAVIOR_FPS = strlen(CMDSTR_SET_BEHAVIOR_FPS);
const int CMDLEN_SET_SYNC_RATIO = strlen(CMDSTR_SET_SYNC_RATIO);
const int CMDLEN_START_RECORDING = strlen(CMDSTR_START_RECORDING);
const int CMDLEN_STOP_RECORDING = strlen(CMDSTR_STOP_RECORDING);

enum ProtocolOperation {
  OPTO_ON,
  OPTO_OFF,
  PROTOCOL_ENDS,
};

class ProtocolStep {
public:
  unsigned long frameCount;
  ProtocolOperation operation;
  int optoChannel = -1;
  bool isValid = false;

  ProtocolStep(unsigned long frameCount,
               ProtocolOperation operation,
               int optoChannel);
  ProtocolStep(const std::string& protocolStepStr);
  ProtocolStep(const char *protocolStepStr);

  std::string toString() const;
};;

int parseProtocolSequence(const std::string &sequence,
                          std::vector<ProtocolStep> &steps);

#endif