#include <Encoder.h>
#include <Servo.h>
#include "SerialProtocol.h"

// ============================================================
// BUILD OPTIONS
// ============================================================
// Set true only while bench-testing with the drive wheels lifted.
// Serial commands and the communication watchdog are bypassed.
constexpr bool RUN_HARDCODED_TESTS = false;

// ============================================================
// MOTOR DRIVER PINS
// ============================================================
constexpr uint8_t ENA   = 5;  // Left motor PWM
constexpr uint8_t DIR_A = 8;  // Left motor direction
constexpr uint8_t ENB   = 6;  // Right motor PWM
constexpr uint8_t DIR_B = 7;  // Right motor direction

// ============================================================
// ENCODER PINS
// ============================================================
constexpr uint8_t RIGHT_ENC_A = 2;
constexpr uint8_t RIGHT_ENC_B = 12;
constexpr uint8_t LEFT_ENC_A  = 13;
constexpr uint8_t LEFT_ENC_B  = 3;

// ============================================================
// STEERING SERVO PINS AND CALIBRATION
// Standard Servo on an Uno uses Timer1, so motor PWM remains on 5/6.
// Power the steering servos from a suitable external supply and share GND.
// ============================================================
constexpr uint8_t LEFT_STEER_PIN  = 9;
constexpr uint8_t RIGHT_STEER_PIN = 10;

constexpr int16_t LEFT_STEER_NEUTRAL_X10  = 900;  // 90.0 degrees
constexpr int16_t RIGHT_STEER_NEUTRAL_X10 = 900;  // 90.0 degrees

constexpr int16_t STEER_MIN_X10 = 0;
constexpr int16_t STEER_MAX_X10 = 1800;

// ============================================================
// MOTOR / CONTROL SETTINGS
// ============================================================
// 44 counts per motor revolution * 15:1 gearbox = 660 counts/output rev.
constexpr float CPR = 660.0f;

// Estimated no-load output-shaft speed at 8.4 V:
// 500 RPM at 12 V * 8.4/12 = 350 RPM.
constexpr float MAX_RPM_ESTIMATE = 350.0f;

constexpr int16_t PWM_MAX = 255;
constexpr int16_t DEADZONE_MIN = 35;

constexpr unsigned long CONTROL_INTERVAL_MS   = 10;   // 100 Hz PI
constexpr unsigned long SPEED_INTERVAL_MS     = 20;   // 50 Hz RPM estimate
constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;  // 10 Hz telemetry
constexpr unsigned long COMMAND_TIMEOUT_MS    = 300;  // communication watchdog

constexpr float CONTROL_TS_SECONDS =
    static_cast<float>(CONTROL_INTERVAL_MS) / 1000.0f;

// Existing tuned gains.
constexpr float KP_LEFT  = 1.6f;
constexpr float KP_RIGHT = 1.7f;
constexpr float KI_LEFT  = 0.3f;
constexpr float KI_RIGHT = 0.3f;

// Prevent unreasonable integral accumulation even away from saturation.
constexpr float INTEGRAL_LIMIT = 500.0f;

// ============================================================
// OBJECTS
// ============================================================
Encoder leftEncoder(LEFT_ENC_A, LEFT_ENC_B);
Encoder rightEncoder(RIGHT_ENC_A, RIGHT_ENC_B);

Servo leftSteeringServo;
Servo rightSteeringServo;

SerialProtocol protocol(Serial);

// ============================================================
// COMMAND / CONTROL STATE
// ============================================================
float leftTargetRPM = 0.0f;
float rightTargetRPM = 0.0f;

float leftMeasuredRPM = 0.0f;
float rightMeasuredRPM = 0.0f;

float leftIntegral = 0.0f;
float rightIntegral = 0.0f;

int16_t leftSignedPWM = 0;
int16_t rightSignedPWM = 0;

int16_t leftSteeringTargetX10 = LEFT_STEER_NEUTRAL_X10;
int16_t rightSteeringTargetX10 = RIGHT_STEER_NEUTRAL_X10;

uint16_t lastAcceptedSequence = 0;

bool hasValidCommand = false;
bool communicationTimedOut = true;

unsigned long lastValidCommandTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastSpeedTime = 0;
unsigned long lastTelemetryTime = 0;

long lastLeftCountForSpeed = 0;
long lastRightCountForSpeed = 0;

// ============================================================
// OPTIONAL BENCH-TEST SEQUENCE
// ============================================================
struct TestStep {
  int16_t leftRPMX10;
  int16_t rightRPMX10;
  unsigned long durationMs;
};

constexpr TestStep HARDWARE_TESTS[] = {
    { 900,  900, 4000},  // 90 / 90 RPM
    { 550,  550, 4000},  // 55 / 55 RPM
    { 600,  900, 4000},  // 60 / 90 RPM
    { 900,  600, 4000},  // 90 / 60 RPM
    {   0,    0, 1500},  // stop before reversing
    {-400, -400, 4000},  // -40 / -40 RPM
    {   0,    0, 3000}   // stop
};

constexpr size_t HARDWARE_TEST_COUNT =
    sizeof(HARDWARE_TESTS) / sizeof(HARDWARE_TESTS[0]);

size_t activeTestIndex = 0;
unsigned long activeTestStartTime = 0;

// ============================================================
// HELPERS
// ============================================================
int8_t signOf(float value) {
  if (value > 0.0f) return 1;
  if (value < 0.0f) return -1;
  return 0;
}

long readLeftSignedCounts() {
  // Existing hardware orientation requires inversion on the left encoder.
  return -leftEncoder.read();
}

long readRightSignedCounts() {
  return rightEncoder.read();
}

void writeSteeringTargets(int16_t leftX10, int16_t rightX10) {
  leftSteeringTargetX10 =
      constrain(leftX10, STEER_MIN_X10, STEER_MAX_X10);
  rightSteeringTargetX10 =
      constrain(rightX10, STEER_MIN_X10, STEER_MAX_X10);

  const int leftDegrees =
      static_cast<int>((leftSteeringTargetX10 + 5) / 10);
  const int rightDegrees =
      static_cast<int>((rightSteeringTargetX10 + 5) / 10);

  leftSteeringServo.write(leftDegrees);
  rightSteeringServo.write(rightDegrees);
}

void resetControllerState() {
  leftIntegral = 0.0f;
  rightIntegral = 0.0f;
}

void setTargetRPM(float newLeftTarget, float newRightTarget) {
  // Reset each integral whenever its requested direction changes.
  if (signOf(newLeftTarget) != signOf(leftTargetRPM)) {
    leftIntegral = 0.0f;
  }
  if (signOf(newRightTarget) != signOf(rightTargetRPM)) {
    rightIntegral = 0.0f;
  }

  leftTargetRPM =
      constrain(newLeftTarget, -MAX_RPM_ESTIMATE, MAX_RPM_ESTIMATE);
  rightTargetRPM =
      constrain(newRightTarget, -MAX_RPM_ESTIMATE, MAX_RPM_ESTIMATE);
}

float rpmToFeedforwardPWM(float targetRPM) {
  if (fabs(targetRPM) < 0.5f) {
    return 0.0f;
  }

  float magnitude =
      fabs(targetRPM) * static_cast<float>(PWM_MAX) / MAX_RPM_ESTIMATE;

  magnitude = constrain(
      magnitude,
      static_cast<float>(DEADZONE_MIN),
      static_cast<float>(PWM_MAX));

  return targetRPM > 0.0f ? magnitude : -magnitude;
}

int16_t calculatePIOutput(
    float targetRPM,
    float measuredRPM,
    float kp,
    float ki,
    float &integral) {

  if (fabs(targetRPM) < 0.5f) {
    integral = 0.0f;
    return 0;
  }

  const float error = targetRPM - measuredRPM;

  float integralCandidate = integral + error * CONTROL_TS_SECONDS;
  integralCandidate =
      constrain(integralCandidate, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  const float feedforward = rpmToFeedforwardPWM(targetRPM);
  const float unsaturated =
      feedforward + kp * error + ki * integralCandidate;

  // The signed output supports reverse, but the controller is not allowed
  // to reverse polarity merely to correct overspeed. Its sign follows the
  // requested target direction.
  const float minimumOutput =
      targetRPM > 0.0f ? static_cast<float>(DEADZONE_MIN)
                       : -static_cast<float>(PWM_MAX);

  const float maximumOutput =
      targetRPM > 0.0f ? static_cast<float>(PWM_MAX)
                       : -static_cast<float>(DEADZONE_MIN);

  const float saturated =
      constrain(unsaturated, minimumOutput, maximumOutput);

  // Conditional-integration anti-windup at both the PWM ceiling and the
  // usable-motion floor.
  const bool pushingAboveMaximum =
      unsaturated > maximumOutput && error > 0.0f;
  const bool pushingBelowMinimum =
      unsaturated < minimumOutput && error < 0.0f;

  if (!pushingAboveMaximum && !pushingBelowMinimum) {
    integral = integralCandidate;
  }

  return static_cast<int16_t>(lroundf(saturated));
}

void applySignedMotor(
    uint8_t pwmPin,
    uint8_t directionPin,
    int16_t signedPWM) {

  const int16_t limited =
      constrain(
          signedPWM,
          static_cast<int16_t>(-PWM_MAX),
          static_cast<int16_t>(PWM_MAX));

  if (limited > 0) {
    digitalWrite(directionPin, HIGH);
    analogWrite(pwmPin, limited);
  } else if (limited < 0) {
    digitalWrite(directionPin, LOW);
    analogWrite(pwmPin, -limited);
  } else {
    analogWrite(pwmPin, 0);
  }
}

void applyMotorOutputs() {
  applySignedMotor(ENA, DIR_A, leftSignedPWM);
  applySignedMotor(ENB, DIR_B, rightSignedPWM);
}

void stopMotorsImmediately() {
  leftSignedPWM = 0;
  rightSignedPWM = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void enterCommunicationSafeState() {
  setTargetRPM(0.0f, 0.0f);
  resetControllerState();
  stopMotorsImmediately();

  // Watchdog neutralizes both steering targets as requested.
  writeSteeringTargets(
      LEFT_STEER_NEUTRAL_X10,
      RIGHT_STEER_NEUTRAL_X10);

  communicationTimedOut = true;
  hasValidCommand = false;
}

void applyDriveCommand(const DriveCommand &command, unsigned long now) {
  lastAcceptedSequence = command.sequence;

  setTargetRPM(
      command.leftRPMX10 / 10.0f,
      command.rightRPMX10 / 10.0f);

  writeSteeringTargets(
      command.leftSteeringX10,
      command.rightSteeringX10);

  lastValidCommandTime = now;
  hasValidCommand = true;
  communicationTimedOut = false;
}

void updateCommunication(unsigned long now) {
  DriveCommand receivedCommand;

  // poll() drains currently available bytes and never waits for more data.
  if (protocol.poll(receivedCommand)) {
    applyDriveCommand(receivedCommand, now);
  }
}

void updateCommunicationWatchdog(unsigned long now) {
  if (RUN_HARDCODED_TESTS) {
    return;
  }

  if (!hasValidCommand ||
      now - lastValidCommandTime > COMMAND_TIMEOUT_MS) {
    if (!communicationTimedOut) {
      enterCommunicationSafeState();
    }
  }
}

void updateRPMEstimate(unsigned long now) {
  if (now - lastSpeedTime < SPEED_INTERVAL_MS) {
    return;
  }

  const unsigned long elapsedMs = now - lastSpeedTime;
  lastSpeedTime = now;

  const long currentLeftCount = readLeftSignedCounts();
  const long currentRightCount = readRightSignedCounts();

  const long deltaLeft =
      currentLeftCount - lastLeftCountForSpeed;
  const long deltaRight =
      currentRightCount - lastRightCountForSpeed;

  lastLeftCountForSpeed = currentLeftCount;
  lastRightCountForSpeed = currentRightCount;

  const float elapsedSeconds = elapsedMs / 1000.0f;

  leftMeasuredRPM =
      (deltaLeft * 60.0f) / (CPR * elapsedSeconds);
  rightMeasuredRPM =
      (deltaRight * 60.0f) / (CPR * elapsedSeconds);
}

void updatePIControllers(unsigned long now) {
  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }

  // Preserve a stable nominal 100 Hz schedule without a blocking delay.
  lastControlTime += CONTROL_INTERVAL_MS;

  leftSignedPWM = calculatePIOutput(
      leftTargetRPM,
      leftMeasuredRPM,
      KP_LEFT,
      KI_LEFT,
      leftIntegral);

  rightSignedPWM = calculatePIOutput(
      rightTargetRPM,
      rightMeasuredRPM,
      KP_RIGHT,
      KI_RIGHT,
      rightIntegral);

  applyMotorOutputs();
}

void sendTelemetryIfDue(unsigned long now) {
  if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastTelemetryTime = now;

  uint8_t status = protocol.getErrorFlags();
  if (communicationTimedOut) {
    status |= STATUS_COMMS_TIMEOUT;
  }

  TelemetryData telemetry;
  telemetry.sequence = lastAcceptedSequence;
  telemetry.leftMeasuredRPMX10 =
      static_cast<int16_t>(lroundf(leftMeasuredRPM * 10.0f));
  telemetry.rightMeasuredRPMX10 =
      static_cast<int16_t>(lroundf(rightMeasuredRPM * 10.0f));
  telemetry.leftSignedPWM = leftSignedPWM;
  telemetry.rightSignedPWM = rightSignedPWM;
  telemetry.leftTicks = readLeftSignedCounts();
  telemetry.rightTicks = readRightSignedCounts();
  telemetry.status = status;

  // If the TX buffer cannot accept the whole frame, skip this telemetry
  // cycle rather than delaying the 100 Hz PI loop.
  if (protocol.sendTelemetry(telemetry)) {
    protocol.clearErrorFlags();
  }
}

void updateHardcodedTests(unsigned long now) {
  if (!RUN_HARDCODED_TESTS) {
    return;
  }

  if (activeTestStartTime == 0) {
    activeTestStartTime = now;
  }

  const TestStep &step = HARDWARE_TESTS[activeTestIndex];

  setTargetRPM(
      step.leftRPMX10 / 10.0f,
      step.rightRPMX10 / 10.0f);

  writeSteeringTargets(
      LEFT_STEER_NEUTRAL_X10,
      RIGHT_STEER_NEUTRAL_X10);

  communicationTimedOut = false;
  hasValidCommand = true;

  if (now - activeTestStartTime >= step.durationMs) {
    activeTestIndex = (activeTestIndex + 1) % HARDWARE_TEST_COUNT;
    activeTestStartTime = now;
    resetControllerState();
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  stopMotorsImmediately();

  leftSteeringServo.attach(LEFT_STEER_PIN);
  rightSteeringServo.attach(RIGHT_STEER_PIN);
  writeSteeringTargets(
      LEFT_STEER_NEUTRAL_X10,
      RIGHT_STEER_NEUTRAL_X10);

  leftEncoder.write(0);
  rightEncoder.write(0);

  const unsigned long now = millis();
  lastControlTime = now;
  lastSpeedTime = now;
  lastTelemetryTime = now;
  lastValidCommandTime = now;

  lastLeftCountForSpeed = readLeftSignedCounts();
  lastRightCountForSpeed = readRightSignedCounts();

  // Remain stopped and centered until the first valid CRC-protected command.
  communicationTimedOut = true;
  hasValidCommand = false;
}

void loop() {
  const unsigned long now = millis();

  if (RUN_HARDCODED_TESTS) {
    updateHardcodedTests(now);
  } else {
    updateCommunication(now);
    updateCommunicationWatchdog(now);
  }

  updateRPMEstimate(now);
  updatePIControllers(now);
  sendTelemetryIfDue(now);
}
