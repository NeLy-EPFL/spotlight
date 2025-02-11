/**
 * This Arduino program synchronizes the behavior recording camera, the muscle
 * recording camera, the infrared illumination light, and the blue GCaMP
 * excitation LED by sending digital trigger signals to the appropriate cameras
 * and light controllers.
 */

#include <unordered_map>


enum Peripheral {
  BEHAVIOR_CAMERA,
  IR_ILLUMINATION,
  MUSCLE_CAMERA,
  BLUE_EXCITATION,
};

enum Color {
  RED,
  GREEN,
  BLUE,
};

enum Status {
  IDLE,
  RECORDING,
  WARNING,
  ERROR,
};

enum PeripheralState {
  INACTIVE,
  ACTIVE,
};


// Pin assignment ============================================================
std::unordered_map<Peripheral, int> controlPins = {
  { BEHAVIOR_CAMERA, 2 },
  { IR_ILLUMINATION, 3 },
  { MUSCLE_CAMERA, 4 },
  { BLUE_EXCITATION, 5 },
};

std::unordered_map<Peripheral, int> indicatorPins = {
  { BEHAVIOR_CAMERA, 6 },
  { IR_ILLUMINATION, 7 },
  { MUSCLE_CAMERA, 8 },
  { BLUE_EXCITATION, 9 },
};

std::unordered_map<Color, int> statusLEDPins = {
  { RED, 10 },
  { GREEN, 11 },
  { BLUE, 12 },
};


// Active voltage ============================================================
// Whether the peripheral should be active when the voltage is high or low
std::unordered_map<Peripheral, int> activeVoltage = {
  { BEHAVIOR_CAMERA, HIGH },
  { IR_ILLUMINATION, HIGH },
  { MUSCLE_CAMERA, HIGH },
  { BLUE_EXCITATION, HIGH },
};


// Warning parameters ========================================================
// If the microcontroller does less than updateRateWarningLevel updates per
// second, something is probably slowing it down, and we should be careful.
int updateRateWarningLevel = 100000;
// If (recording FPS * exposure/active time) > duty_cycle_warning_level, we
// should warn the user that this is probably not feasible on the hardware.
float dutyCycleWarningLevel = 0.9;


// Recording parameters ======================================================
// The frame rate of the behavior camera is defined by its reciprocal
// (behaviorFrequency = 1,000,000 / behaviorRecordingIntervalUs).
int behaviorRecordingIntervalUs;
// The frame rate of the behavior is given as a factor of the behavior
// recording frequency. This is to keep the cameras always in sync
// (muscleFrequency = behaviorFrequency / muscleRecordingIntervalMultiplier)
// and robust against realistic clock time (if the light is turned on a few
// microseconds later than it should, and the last active is set to the
// current time, the cameras will both run a tiny bit slower than the
// specified rate, but the amount of delay is not necessarily constant between
// the cameras).
int muscleRecordingIntervalMultiplier;
// Exposure time, or amount of time illumination is on
int behaviorExposureDurationUs;
int muscleExposureDurationUs;


// Timers and state variables ================================================
unsigned long iterCounter = 0;
unsigned long lastIterCounterUpdateUs = 0;

unsigned long lastBehaviorActiveStartTimeUs = 0;
unsigned long lastMuscleActiveStartTimeUs = 0;
unsigned long numBehaviorRecordingCycles = 0;

Status currentStatus;


int changePeripheralState(
  Peripheral peripheral,
  PeripheralState targetState
) {
  // Find out which pins to use
  int controlPin = controlPins.at(peripheral);
  int indicatorPin = indicatorPins.at(peripheral);

  // Send out the appropriate digital to the peripheral device
  int targetVoltage = (targetState == ACTIVE)
                        ? activeVoltage.at(peripheral)
                        : !activeVoltage.at(peripheral);
  digitalWrite(controlPin, targetVoltage);

  // Update LED indicator
  digitalWrite(indicatorPin, (targetState == ACTIVE) ? HIGH : LOW);

  return targetVoltage;
}


void setup() {
  Serial.begin(9600);  // open the serial port at 9600 bps

  // Initialize pin mode
  for (const auto& [peripheral, pin] : controlPins) {
    pinMode(pin, OUTPUT);
  }
  for (const auto& [peripheral, pin] : indicatorPins) {
    pinMode(pin, OUTPUT);
  }
  for (const auto& [color, pin] : statusLEDPins) {
    pinMode(pin, OUTPUT);
  }
  currentStatus = IDLE;

  // TODO: Read recording specs from USB
  behaviorRecordingIntervalUs = 200000;
  muscleRecordingIntervalMultiplier = 5;
  behaviorExposureDurationUs = 10000;
  muscleExposureDurationUs = 30000;


  // Check timing validity
  checkTimingConfigValidity(
    behaviorRecordingIntervalUs, behaviorExposureDurationUs
  );
  checkTimingConfigValidity(
    behaviorRecordingIntervalUs * muscleRecordingIntervalMultiplier,
    muscleExposureDurationUs
  );
}


int checkTimingConfigValidity(int intervalUs, int durationUs) {
  float dutyCycle = durationUs / intervalUs;
  if (dutyCycle > 1) {
    Serial.println(
      "Invalid camera configuration: duty cycle above 100%."
    );
    currentStatus = ERROR;
    return 1;
  }

  if (dutyCycle >= dutyCycleWarningLevel) {
    Serial.println(
      "Warning: Camera's duty cycle is very close to 100%. "
      "Pay attention to real FPS; it might be lower than the specified "
      "value due to hardware refractory periods and delays."
    );
    currentStatus = WARNING;
    return 2;
  }

  return 0;
}


void loop() {
  // Update status LED
  switch (currentStatus) {
    case IDLE:
      digitalWrite(statusLEDPins.at(RED), LOW);
      digitalWrite(statusLEDPins.at(GREEN), HIGH);
      digitalWrite(statusLEDPins.at(BLUE), LOW);
      break;
    case RECORDING:
      digitalWrite(statusLEDPins.at(RED), LOW);
      digitalWrite(statusLEDPins.at(GREEN), LOW);
      digitalWrite(statusLEDPins.at(BLUE), HIGH);
      break;
    case WARNING:
      digitalWrite(statusLEDPins.at(RED), HIGH);
      digitalWrite(statusLEDPins.at(GREEN), HIGH);
      digitalWrite(statusLEDPins.at(BLUE), LOW);
    case ERROR:
      digitalWrite(statusLEDPins.at(RED), HIGH);
      digitalWrite(statusLEDPins.at(GREEN), LOW);
      digitalWrite(statusLEDPins.at(BLUE), LOW);
      break;
  }

  unsigned long currentTime = micros();

  // Monitor controlle speed
  iterCounter += 1;
  if (currentTime - lastIterCounterUpdateUs >= 1000000) {
    if (iterCounter < updateRateWarningLevel) {
      Serial.printf(
        "Warning: The microcontroller that generates the trigger signals has "
        "a low update rate of %d Hz. Something might be slowing it down.\n",
        iterCounter
      );
      currentStatus = WARNING;
    }
    
    lastIterCounterUpdateUs = currentTime;
    iterCounter = 0;
  }

  int timeSinceBehaviorActive = currentTime - lastBehaviorActiveStartTimeUs;
  int timeSinceMuscleActive = currentTime - lastMuscleActiveStartTimeUs;

  // Turn-off logic
  // This is easy because a few dozen microseconds of offset doesn't hurt.
  // We can handle behavior and muscle cameras separately.
  if (timeSinceBehaviorActive >= behaviorExposureDurationUs) {
    changePeripheralState(BEHAVIOR_CAMERA, INACTIVE);
    changePeripheralState(IR_ILLUMINATION, INACTIVE);
  }
  if (timeSinceMuscleActive >= muscleExposureDurationUs ) {
    changePeripheralState(MUSCLE_CAMERA, INACTIVE);
    changePeripheralState(BLUE_EXCITATION, INACTIVE);
  }

  // Turn-on logic
  // To keep the cameras always in sync, we must start exposure at the same
  // iteration (ie. clock time) for both camera (if they both need to be
  // turned on).
  if (timeSinceBehaviorActive >= behaviorRecordingIntervalUs) {
    changePeripheralState(BEHAVIOR_CAMERA, ACTIVE);
    changePeripheralState(IR_ILLUMINATION, ACTIVE);
    lastBehaviorActiveStartTimeUs = currentTime;

    if (numBehaviorRecordingCycles % muscleRecordingIntervalMultiplier == 0) {
      changePeripheralState(MUSCLE_CAMERA, ACTIVE);
      changePeripheralState(BLUE_EXCITATION, ACTIVE);
      lastMuscleActiveStartTimeUs = currentTime;
    }

    numBehaviorRecordingCycles += 1;
  }

  currentStatus = RECORDING;
}