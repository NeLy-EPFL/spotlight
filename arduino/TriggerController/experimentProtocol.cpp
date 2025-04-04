#include "experimentProtocol.hpp"

ProtocolStep::ProtocolStep(unsigned long frameCount,
                           ProtocolOperation operation,
                           int optoChannel)
    : frameCount(frameCount), operation(operation), optoChannel(optoChannel) {}

ProtocolStep::ProtocolStep(const std::string& protocolStepStr)
{
  std::istringstream tokenStream(protocolStepStr);
  std::string frameStr, channelStr, opStr;

  if (!std::getline(tokenStream, frameStr, '/') ||
      !std::getline(tokenStream, channelStr, '/') ||
      !std::getline(tokenStream, opStr, '/'))
  {
    isValid = false;
    return;
  }

  // Parse frame number
  frameCount = std::stoul(frameStr);

  // Check if it's a STOP command
  if (channelStr == "x" && opStr == "stop")
  {
    operation = PROTOCOL_ENDS;
    optoChannel = -1;
    isValid = true;
    return;
  }

  // Ensure channel is in the format "chX"
  if (channelStr.substr(0, 2) != "ch")
  {
    isValid = false;
    return;
  }

  optoChannel = std::stoi(channelStr.substr(2));

  // Determine operation
  if (opStr == "on")
  {
    operation = OPTO_ON;
    isValid = true;
  }
  else if (opStr == "off")
  {
    operation = OPTO_OFF;
    isValid = true;
  }
  else
  {
    isValid = false;
    return;
  }
}

ProtocolStep::ProtocolStep(const char *protocolStepStr)
    : ProtocolStep(std::string(protocolStepStr)) {}

std::string ProtocolStep::toString() const {
  if (operation == PROTOCOL_ENDS) {
    return std::to_string(frameCount) + "/x/stop";
  }
  return std::to_string(frameCount) + 
    "/ch" + std::to_string(optoChannel) + "/" +
    (operation == OPTO_ON ? "on" : "off");
}

int parseProtocolSequence(const std::string &sequence,
                          std::vector<ProtocolStep> &steps)
{
  steps.clear(); // Ensure the vector is empty before parsing

  if (sequence == ";")
  {
    return 0;
  }

  std::istringstream stream(sequence);
  std::string command;

  int commandCount = 0;
  while (std::getline(stream, command, ';'))
  {
    if (command.empty()) {
      return -1; // Return error if any command is empty
    }

    ProtocolStep step(command);
    if (!step.isValid)
    {
      return -1; // Return error if any step is invalid
    }
    steps.push_back(step);
    commandCount++;
  }

  return commandCount;
  // Return the number of valid commands parsed
}