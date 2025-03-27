#ifndef ARDUINO_COMMUNICATION_H
#define ARDUINO_COMMUNICATION_H

#include <string>
#include <vector>
#include "parserAndFormatter.hpp"

bool initializeArduinoCommunication(const std::string &portName,
                                    int baudRate = 115200);
void shutdownArduinoCommunication();

int setBehaviorPeriod(int behaviorPeriodUs, int timeoutUs);
int setSyncRatio(int syncRatio, int timeoutUs);
int setBehaviorExposureTime(int exposureTimeUs, int timeoutUs);
int setMuscleExposureTime(int exposureTimeUs, int timeoutUs);
int startTriggering(ProtocolStep *protocolSteps, int timeoutUs);
int stopTriggering(int timeoutUs);
bool incomingArduinoMessageAvailable();
std::string readArduinoMessageIfAvailable();
std::string waitForArduinoMessage(int timeoutUs);

#endif // ARDUINO_COMMUNICATION_H