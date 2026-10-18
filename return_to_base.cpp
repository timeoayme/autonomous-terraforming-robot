// Standalone back-to-base emergency-return test.
// Localizes on a random RFID tag, asks the server for x/y, then returns to the base exit (9,7).
// Uses black-line PID tracking while y <= 4. For y > 4, uses IMU yaw hold. RFID cards correct nodes.
// Build/upload with: pio run -e back_to_base -t upload

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include "ICM_20948.h"
#include <MiniMessenger.h>
#include "secrets.h"

const uint8_t LOWER_MOTORON_ADDR = 0x11;
const uint8_t UPPER_MOTORON_ADDR = 0x10;

MotoronI2C mcLower(LOWER_MOTORON_ADDR);
MotoronI2C mcUpper(UPPER_MOTORON_ADDR);

const uint8_t RFID_I2C_ADDR = 0x28;
const uint8_t RFID_UNUSED_RESET_PIN = 0xFF;
MFRC522_I2C mfrc522(RFID_I2C_ADDR, RFID_UNUSED_RESET_PIN, &Wire1);
MiniMessenger serverMessenger;

ICM_20948_I2C imu;
bool imuAd0Val = true;
float gyroZBias = 0.0;
float yawDeg = 0.0;
unsigned long lastUpdateMicros = 0;

// Lower shield: RF/RR/LR on M1/M2/M3. Upper shield: LF on M1.
const uint8_t RF = 1;
const uint8_t RR = 2;
const uint8_t LR = 3;
const uint8_t LF = 1;

// Current robot motor polarity for physical forward motion.
const int8_t FWD_RF = 1;
const int8_t FWD_RR = -1;
const int8_t FWD_LR = 1;
const int8_t FWD_LF = 1;

// Encoder order is RF, RR, LR, LF.
const uint8_t ENCODER_COUNT = 4;
const uint8_t ENC_A[ENCODER_COUNT] = {18, 2, 4, 8};
const uint8_t ENC_B[ENCODER_COUNT] = {19, 3, 5, 9};

// Reflectance board using only OUT1/OUT3/OUT5/OUT7/OUT9, right to left.
const uint8_t SENSOR_COUNT = 5;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {22, 24, 26, 28, 31};
const char *SENSOR_NAMES[SENSOR_COUNT] = {"O1", "O3", "O5", "O7", "O9"};

const uint8_t REVIVE_BUTTON_PIN = 32;
const uint8_t KILL_BUTTON_PIN = 45;
const uint8_t LED_RED_PIN = 36;
const uint8_t LED_GREEN_PIN = 38;

const uint8_t US_FRONT_ECHO_PIN = 37;
const uint8_t US_FRONT_TRIG_PIN = 39;
const uint8_t US_LEFT_ECHO_PIN = 52;
const uint8_t US_LEFT_TRIG_PIN = 44;
const uint8_t US_RIGHT_ECHO_PIN = 46;
const uint8_t US_RIGHT_TRIG_PIN = 48;

const int8_t ARENA_MIN_COORD = 0;
const int8_t ARENA_MAX_COORD = 9;
const int8_t ARENA_ENTRY_X = 9;
const int8_t ARENA_ENTRY_Y = 3;
const int8_t ARENA_EXIT_X = 9;
const int8_t ARENA_EXIT_Y = 7;
const int8_t FIRST_SERVER_TAG_X = 9; // User-provided: entrance / first slope tag is (9,3).
const int8_t FIRST_SERVER_TAG_Y = 3;
const int8_t GRID_HEADING_NORTH = 0; // +Y, initial entry heading.
const int8_t GRID_HEADING_EAST = 1;  // +X.
const int8_t GRID_HEADING_SOUTH = 2; // -Y.
const int8_t GRID_HEADING_WEST = 3;  // -X.

const int16_t MAX_SPEED = 1100;
const int16_t RANDOM_FORWARD_SPEED = 360;
const int16_t ENTRY_CORRIDOR_LINE_SPEED = 220;
const int16_t ENTRY_CORRIDOR_WALL_SPEED = 210;
const int16_t ENTRY_CORRIDOR_MIN_SPEED = 190;
const int16_t ENTRY_CORRIDOR_ESCAPE_SPEED = 220;
const int16_t ENTRY_CORRIDOR_MAX_CORRECTION = 70;
const float ENTRY_CORRIDOR_WALL_KP = 10.0;
const float ENTRY_CORRIDOR_WALL_KD = 3.0;
const float ENTRY_CORRIDOR_CRITICAL_CM = 5.5;
const float ENTRY_CORRIDOR_SINGLE_WALL_CM = 9.0;
const float ENTRY_CORRIDOR_FRONT_BLOCK_CM = 11.0;
const unsigned long ENTRY_CORRIDOR_ESCAPE_MS = 240;
const int16_t RETURN_SPEED = 360;
const int16_t LINE_RETURN_SPEED = 330;
const int16_t LINE_LOST_YAW_SPEED = 300;
const int16_t TUNNEL_SPEED = 430;
const int16_t TUNNEL_MIN_SPEED = 320;
const int16_t BASE_RIGHT_PRIORITY_SPEED = 300;
const int16_t RETURN_TURN_FAST_SPEED = 560;
const int16_t RETURN_TURN_MID_SPEED = 500;
const int16_t RETURN_TURN_FINE_SPEED = 340;
const int16_t RETURN_TURN_RELIEF_SPEED = 420;
const int16_t RETURN_MAX_CORRECTION = 130;
const float RETURN_YAW_KP = 16.0;
const float RETURN_TURN_TOLERANCE_DEG = 3.0;
const float RETURN_TURN_MID_REMAINING_DEG = 28.0;
const float RETURN_TURN_FINE_REMAINING_DEG = 9.0;
const float RETURN_TURN_SEGMENT_DEG = 12.0;
const uint16_t RETURN_TURN_SEGMENT_STOP_MS = 70;
const uint16_t RETURN_TURN_RELIEF_PULSE_MS = 80;
const uint16_t RETURN_TURN_SETTLE_MS = 180;
const uint16_t RETURN_TURN_TIMEOUT_MS = 14000;
const uint8_t START_FORWARD_NODES = 1;
const unsigned long START_RFID_IGNORE_MS = 1;
const unsigned long NODE_TIMEOUT_MS = 6500;
const long NODE_ENCODER_TICKS = 160;
const unsigned int SENSOR_TIMEOUT_US = 3000;
const bool BLACK_LINE_IS_HIGH = true;
const unsigned int LINE_THRESHOLD = 280;
const unsigned int LINE_SIGNAL_FLOOR = 240;
const int LINE_MIN_SIGNAL_TOTAL = 20;
const float LINE_PID_KP = 0.22;
const float LINE_PID_KI = 0.03;
const float LINE_PID_KD = 0.006;
const float LINE_PID_INTEGRAL_LIMIT = 3000.0;
const int16_t LINE_PID_MAX_CORRECTION = 300;
const unsigned long RFID_CHECK_INTERVAL_MS = 80;
const unsigned long RFID_RETRY_INTERVAL_MS = 1000;
const unsigned long NODE_REPEAT_MS = 900;
const unsigned long STATUS_INTERVAL_MS = 250;
const unsigned long SENSOR_MONITOR_INTERVAL_MS = 100;
const unsigned long STARTUP_BLINK_MS = 300;
const unsigned long RUN_TIME_LIMIT_MS = 60000;
const bool USE_SERVER_COORDINATES = true;
const bool USE_FIRST_TAG_COORD_FALLBACK = false;
const char *SERVER_TARGET_BOARD = "server";
const unsigned long SERVER_REGISTER_INTERVAL_MS = 3000;
const unsigned long SERVER_REQUEST_COOLDOWN_MS = 700;
const unsigned long LEG_SERVER_COORD_WAIT_MS = 1400;
const float FRONT_OBSTACLE_CM = 13.0;
const float FRONT_PINNED_CM = 6.0;
const int16_t OBSTACLE_BACK_AWAY_SPEED = 240;
const unsigned long OBSTACLE_BACK_AWAY_MS = 550;
const float SIDE_WALL_BLOCK_CM = 7.0;
const uint8_t MAX_OBSTACLE_RECURSION_DEPTH = 3;
const unsigned long OBSTACLE_STEP_TIMEOUT_MS = 5000;
const uint8_t OBSTACLE_SIDE_STEP_NODES = 1;
const uint8_t OBSTACLE_FORWARD_NODES = 2;
const float WALL_VALID_MAX_CM = 90.0;
const float WALL_TARGET_CM = 15.0;
const float WALL_CRITICAL_CM = 4.5;
const float WALL_KP = 22.0;
const float WALL_KD = 10.0;
const int16_t WALL_MAX_CORRECTION = 180;
const unsigned long TUNNEL_TO_BASE_MS = 4500;
const uint8_t RIGHT_PRIORITY_T_ACTIVE_MIN = 4;
const unsigned long FIRST_TAG_SERVER_WAIT_MS = 3000;

enum BackState
{
  RANDOM_FORWARD,
  ALIGN_X,
  DRIVE_X,
  ALIGN_Y,
  DRIVE_Y,
  OBSTACLE_AVOID,
  TUNNEL_TO_BASE,
  BASE_RIGHT_PRIORITY,
  DONE,
  FAILED
};

enum ObstaclePhase
{
  OBS_IDLE,
  OBS_BACK_AWAY,
  OBS_TURN_OUT,
  OBS_SIDE_STEP_OUT,
  OBS_TURN_FORWARD,
  OBS_PASS_OBSTACLE,
  OBS_TURN_BACK,
  OBS_SIDE_STEP_BACK,
  OBS_TURN_ORIGINAL
};

enum ReturnTurnPhase
{
  RETURN_TURN_APPROACH,
  RETURN_TURN_SEGMENT_STOP,
  RETURN_TURN_RELIEF_FORWARD,
  RETURN_TURN_RELIEF_BACKWARD,
  RETURN_TURN_SETTLE,
  RETURN_TURN_FINE_CORRECT
};

BackState state = RANDOM_FORWARD;
unsigned long stateStartMs = 0;
unsigned long runStartMs = 0;
unsigned long lastRFIDCheckMs = 0;
unsigned long lastRFIDRetryMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastSensorMonitorMs = 0;
unsigned long turnSettleStartMs = 0;
unsigned long nodeStartMs = 0;
unsigned long startupLastBlinkMs = 0;
unsigned long returnTurnPhaseStartMs = 0;
unsigned long lastReturnTurnPrintMs = 0;
bool startupWaiting = true;
bool startupRedOn = true;
bool ignoreD45UntilReleased = false;
bool killStopped = true;
bool rfidReady = false;
bool serverMessengerStarted = false;
bool serverLastConnected = false;
unsigned long serverLastRegisterMs = 0;
unsigned long lastServerRfidRequestMs = 0;
bool serverCoordinateKnown = false;
bool firstTagFallbackUsed = false;
bool waitingForFirstServerCoordinate = false;
unsigned long firstServerCoordWaitStartMs = 0;
bool waitingForLegServerCoordinate = false;
unsigned long legServerCoordWaitStartMs = 0;
char lastRFIDTagId[24] = "";
bool routeReplanPending = false;
bool returnTurnActive = false;
bool returnTurnNextReliefForward = true;
ReturnTurnPhase returnTurnPhase = RETURN_TURN_APPROACH;
float returnTurnStartYawDeg = 0.0;
float returnTurnTargetDeltaDeg = 0.0;
float returnTurnSegmentTargetDeg = 0.0;

unsigned int sensorValues[SENSOR_COUNT] = {0, 0, 0, 0, 0};
bool sensorOnLine[SENSOR_COUNT] = {false, false, false, false, false};
float linePidIntegral = 0.0;
float linePidLastError = 0.0;
unsigned long linePidLastMs = 0;
bool linePidHasLast = false;
bool lastDriveUsedLine = false;
bool entryCorridorUsingWall = false;
int8_t entryCorridorEscapeDirection = 0;
unsigned long entryCorridorEscapeUntilMs = 0;
unsigned long lastEntryGuidePrintMs = 0;

volatile long encoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
bool lastEncoderA[ENCODER_COUNT] = {HIGH, HIGH, HIGH, HIGH};
long nodeStartEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
long lastStatusEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};

bool haveLastNodeUid = false;
uint32_t lastNodeUid = 0;
unsigned long lastNodeUidMs = 0;
bool haveLastServerCoordinate = false;
int8_t lastServerX = ARENA_ENTRY_X;
int8_t lastServerY = ARENA_ENTRY_Y;

int8_t robotGridX = ARENA_ENTRY_X;
int8_t robotGridY = ARENA_ENTRY_Y;
int8_t robotGridHeading = GRID_HEADING_NORTH;
float gridHeadingZeroYawDeg = 0.0;
bool positionKnown = true;

int8_t targetHeading = GRID_HEADING_NORTH;
float targetYawDeg = 0.0;
uint8_t targetNodes = 0;
uint8_t nodesTravelled = 0;
uint8_t randomNodesCounted = 0;
BackState obstacleResumeState = RANDOM_FORWARD;
ObstaclePhase obstaclePhase = OBS_IDLE;
unsigned long obstaclePhaseStartMs = 0;
int8_t obstacleTurnDirection = 1;
uint8_t obstacleDepth = 0;
float obstacleTurnStartYawDeg = 0.0;
uint8_t obstacleNodeTarget = 0;
uint8_t obstacleNodesTravelled = 0;
long obstacleNodeStartEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
float wallLastError = 0.0;
unsigned long wallLastMs = 0;
bool wallHasLast = false;

void planNextLeg();
void startObstacleAvoidance(const char *source);
void obstacleAvoidStep();
void alignStep(BackState driveState);
void maybePlanAfterServerCoordinate(const char *source);
void setTargetHeading(int8_t heading);

const char *stateName(BackState value)
{
  switch (value)
  {
    case RANDOM_FORWARD: return "RANDOM_FORWARD";
    case ALIGN_X: return "ALIGN_X";
    case DRIVE_X: return "DRIVE_X";
    case ALIGN_Y: return "ALIGN_Y";
    case DRIVE_Y: return "DRIVE_Y";
    case OBSTACLE_AVOID: return "OBSTACLE_AVOID";
    case TUNNEL_TO_BASE: return "TUNNEL_TO_BASE";
    case BASE_RIGHT_PRIORITY: return "BASE_RIGHT_PRIORITY";
    case DONE: return "DONE";
    case FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

const char *obstaclePhaseName(ObstaclePhase phase)
{
  switch (phase)
  {
    case OBS_IDLE: return "IDLE";
    case OBS_BACK_AWAY: return "BACK_AWAY";
    case OBS_TURN_OUT: return "TURN_OUT";
    case OBS_SIDE_STEP_OUT: return "SIDE_STEP_OUT";
    case OBS_TURN_FORWARD: return "TURN_FORWARD";
    case OBS_PASS_OBSTACLE: return "PASS_OBSTACLE";
    case OBS_TURN_BACK: return "TURN_BACK";
    case OBS_SIDE_STEP_BACK: return "SIDE_STEP_BACK";
    case OBS_TURN_ORIGINAL: return "TURN_ORIGINAL";
  }
  return "UNKNOWN";
}

const char *headingName(int8_t heading)
{
  switch (heading)
  {
    case GRID_HEADING_NORTH: return "NORTH";
    case GRID_HEADING_EAST: return "EAST";
    case GRID_HEADING_SOUTH: return "SOUTH";
    case GRID_HEADING_WEST: return "WEST";
  }
  return "UNKNOWN";
}

int16_t clampMotorCommand(int16_t speed)
{
  return constrain(speed, -MAX_SPEED, MAX_SPEED);
}

void setMotorSpeeds(int16_t rf, int16_t rr, int16_t lr, int16_t lf)
{
  mcLower.setSpeed(RF, clampMotorCommand(rf));
  mcLower.setSpeed(RR, clampMotorCommand(rr));
  mcLower.setSpeed(LR, clampMotorCommand(lr));
  mcUpper.setSpeed(LF, clampMotorCommand(lf));
}

void stopAllMotors()
{
  setMotorSpeeds(0, 0, 0, 0);
}

void copyText(char *destination, size_t destinationSize, const char *source)
{
  if (destinationSize == 0)
  {
    return;
  }
  if (source == nullptr)
  {
    destination[0] = '\0';
    return;
  }
  strncpy(destination, source, destinationSize - 1);
  destination[destinationSize - 1] = '\0';
}

bool extractField(const char *message, const char *key, char *out, size_t outSize)
{
  if (message == nullptr || key == nullptr || out == nullptr || outSize == 0)
  {
    return false;
  }

  char pattern[24];
  snprintf(pattern, sizeof(pattern), "%s=", key);
  const char *start = strstr(message, pattern);
  if (start == nullptr)
  {
    out[0] = '\0';
    return false;
  }

  start += strlen(pattern);
  size_t len = 0;
  while (start[len] != '\0' && start[len] != ' ' && len < outSize - 1)
  {
    out[len] = start[len];
    len++;
  }
  out[len] = '\0';
  return len > 0;
}

bool parseIntField(const char *message, const char *key, int &value)
{
  char text[16];
  if (!extractField(message, key, text, sizeof(text)))
  {
    return false;
  }
  value = atoi(text);
  return true;
}

bool messageTypeEquals(const char *message, const char *expectedType)
{
  char type[32];
  return extractField(message, "type", type, sizeof(type)) && strcmp(type, expectedType) == 0;
}

void driveBySide(int16_t leftSpeed, int16_t rightSpeed)
{
  setMotorSpeeds(
      rightSpeed * FWD_RF,
      rightSpeed * FWD_RR,
      leftSpeed * FWD_LR,
      leftSpeed * FWD_LF);
}

void driveForward(int16_t speed)
{
  driveBySide(speed, speed);
}

void triggerUltrasonic(uint8_t trigPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin)
{
  triggerUltrasonic(trigPin);
  unsigned long duration = pulseIn(echoPin, HIGH, 22000UL);
  if (duration == 0)
  {
    return -1.0;
  }
  return duration * 0.0343 / 2.0;
}

float frontDistanceCm()
{
  return readUltrasonicCm(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
}

float leftDistanceCm()
{
  return readUltrasonicCm(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);
}

float rightDistanceCm()
{
  return readUltrasonicCm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);
}

bool validWallDistance(float distanceCm)
{
  return distanceCm > 0.0 && distanceCm <= WALL_VALID_MAX_CM;
}

bool frontObstacleDetected()
{
  float frontCm = frontDistanceCm();
  return frontCm > 0.0 && frontCm <= FRONT_OBSTACLE_CM;
}

bool headingSideDistance(int8_t heading, float &distanceCm, float &blockCm)
{
  int8_t relative = (heading - robotGridHeading + 4) % 4;
  if (relative == 0)
  {
    distanceCm = frontDistanceCm();
    blockCm = FRONT_OBSTACLE_CM;
    return true;
  }

  if (relative == 1)
  {
    distanceCm = rightDistanceCm();
    blockCm = SIDE_WALL_BLOCK_CM;
    return true;
  }

  if (relative == 3)
  {
    distanceCm = leftDistanceCm();
    blockCm = SIDE_WALL_BLOCK_CM;
    return true;
  }

  distanceCm = -1.0;
  blockCm = SIDE_WALL_BLOCK_CM;
  return false;
}

bool headingBlockedByWall(int8_t heading)
{
  float distanceCm = -1.0;
  float blockCm = SIDE_WALL_BLOCK_CM;
  if (!headingSideDistance(heading, distanceCm, blockCm))
  {
    return false;
  }

  return distanceCm > 0.0 && distanceCm <= blockCm;
}

void printWallSnapshot(const char *source)
{
  float frontCm = frontDistanceCm();
  float leftCm = leftDistanceCm();
  float rightCm = rightDistanceCm();
  Serial.print("WALL_SCAN | ");
  Serial.print(source);
  Serial.print(" front=");
  Serial.print(frontCm, 1);
  Serial.print("cm left=");
  Serial.print(leftCm, 1);
  Serial.print("cm right=");
  Serial.print(rightCm, 1);
  Serial.print("cm heading=");
  Serial.println(headingName(robotGridHeading));
}

unsigned int readReflectancePin(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);

  unsigned long start = micros();
  while (digitalRead(pin) == HIGH)
  {
    if (micros() - start >= SENSOR_TIMEOUT_US)
    {
      return SENSOR_TIMEOUT_US;
    }
  }

  return (unsigned int)(micros() - start);
}

void readSensors()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorValues[i] = readReflectancePin(SENSOR_PINS[i]);
    bool highMeansLine = sensorValues[i] >= LINE_THRESHOLD;
    sensorOnLine[i] = BLACK_LINE_IS_HIGH ? highMeansLine : !highMeansLine;
  }
}

void resetLinePid()
{
  linePidIntegral = 0.0;
  linePidLastError = 0.0;
  linePidLastMs = millis();
  linePidHasLast = false;
}

bool computeLineError(float &error)
{
  const int sensorWeights[SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000};
  long weightedSum = 0;
  int signalTotal = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    int signal = sensorValues[i] > LINE_SIGNAL_FLOOR
                     ? (int)(sensorValues[i] - LINE_SIGNAL_FLOOR)
                     : 0;
    weightedSum += (long)signal * sensorWeights[i];
    signalTotal += signal;
  }

  if (signalTotal < LINE_MIN_SIGNAL_TOTAL)
  {
    return false;
  }

  error = (float)weightedSum / (float)signalTotal;
  return true;
}

void monitorReflectanceSensors()
{
  if (millis() - lastSensorMonitorMs < SENSOR_MONITOR_INTERVAL_MS)
  {
    return;
  }

  lastSensorMonitorMs = millis();
  readSensors();
  Serial.print("QTR_MON ");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print("(D");
    Serial.print(SENSOR_PINS[i]);
    Serial.print(")=");
    Serial.print(sensorValues[i]);
    Serial.print(sensorOnLine[i] ? "*" : " ");
    if (i + 1 < SENSOR_COUNT)
    {
      Serial.print(" ");
    }
  }
  Serial.print(" | threshold=");
  Serial.print(LINE_THRESHOLD);
  Serial.print(" floor=");
  Serial.println(LINE_SIGNAL_FLOOR);
}

void driveLinePid(float error, int16_t baseSpeed = LINE_RETURN_SPEED)
{
  unsigned long now = millis();
  float dtSec = 0.02;
  if (linePidHasLast && now > linePidLastMs)
  {
    dtSec = (now - linePidLastMs) / 1000.0;
    if (dtSec < 0.005)
    {
      dtSec = 0.005;
    }
    else if (dtSec > 0.1)
    {
      dtSec = 0.1;
    }
  }

  linePidIntegral += error * dtSec;
  linePidIntegral = constrain(linePidIntegral, -LINE_PID_INTEGRAL_LIMIT, LINE_PID_INTEGRAL_LIMIT);
  float derivative = linePidHasLast ? (error - linePidLastError) / dtSec : 0.0;
  float correctionFloat = LINE_PID_KP * error +
                          LINE_PID_KI * linePidIntegral +
                          LINE_PID_KD * derivative;
  int16_t correction = constrain(
      (int16_t)correctionFloat,
      (int16_t)-LINE_PID_MAX_CORRECTION,
      (int16_t) LINE_PID_MAX_CORRECTION);
  int16_t leftSpeed = constrain((int16_t)(baseSpeed - correction), -MAX_SPEED, MAX_SPEED);
  int16_t rightSpeed = constrain((int16_t)(baseSpeed + correction), -MAX_SPEED, MAX_SPEED);

  driveBySide(leftSpeed, rightSpeed);

  linePidLastError = error;
  linePidLastMs = now;
  linePidHasLast = true;
}

void pivotDirection(int direction, int16_t speed)
{
  if (direction == 0)
  {
    stopAllMotors();
    return;
  }

  int16_t rightSign = direction < 0 ? 1 : -1;
  int16_t leftSign = direction < 0 ? -1 : 1;
  setMotorSpeeds(
      rightSign * speed * FWD_RF,
      rightSign * speed * FWD_RR,
      leftSign * speed * FWD_LR,
      leftSign * speed * FWD_LF);
}

void setupMotoron(MotoronI2C &mc, uint8_t m1, uint8_t m2 = 0, uint8_t m3 = 0)
{
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();

  mc.setMaxAcceleration(m1, 700);
  mc.setMaxDeceleration(m1, 850);
  mc.setSpeed(m1, 0);

  if (m2 != 0)
  {
    mc.setMaxAcceleration(m2, 700);
    mc.setMaxDeceleration(m2, 850);
    mc.setSpeed(m2, 0);
  }

  if (m3 != 0)
  {
    mc.setMaxAcceleration(m3, 700);
    mc.setMaxDeceleration(m3, 850);
    mc.setSpeed(m3, 0);
  }
}

bool initIMU()
{
  const bool ad0Values[] = {false, true};
  for (uint8_t i = 0; i < 2; i++)
  {
    imuAd0Val = ad0Values[i];
    Serial.print("IMU | Trying AD0=");
    Serial.println(imuAd0Val ? 1 : 0);
    imu.begin(Wire, imuAd0Val);
    Serial.print("IMU | Status: ");
    Serial.println(imu.statusString());
    if (imu.status == ICM_20948_Stat_Ok)
    {
      return true;
    }
    delay(100);
  }
  return false;
}

void calibrateGyroZ()
{
  Serial.println("IMU | Calibrating gyro Z. Keep still...");
  const int samples = 300;
  const unsigned long timeoutMs = 6000;
  float sum = 0.0;
  int count = 0;
  unsigned long startMs = millis();

  while (count < samples && millis() - startMs < timeoutMs)
  {
    if (imu.dataReady())
    {
      imu.getAGMT();
      sum += imu.gyrZ();
      count++;
    }
    delay(5);
  }

  gyroZBias = count > 0 ? sum / count : 0.0;
  yawDeg = 0.0;
  gridHeadingZeroYawDeg = 0.0;
  Serial.print("IMU | Gyro Z bias=");
  Serial.print(gyroZBias, 6);
  Serial.print(" samples=");
  Serial.println(count);
}

void updateYaw()
{
  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();
  unsigned long nowMicros = micros();
  if (lastUpdateMicros == 0)
  {
    lastUpdateMicros = nowMicros;
    return;
  }

  float dt = (nowMicros - lastUpdateMicros) / 1000000.0;
  lastUpdateMicros = nowMicros;
  if (dt <= 0.0 || dt > 0.25)
  {
    return;
  }

  yawDeg += (imu.gyrZ() - gyroZBias) * dt;
}

float normalizeAngleDeg(float angle)
{
  while (angle > 180.0)
  {
    angle -= 360.0;
  }
  while (angle < -180.0)
  {
    angle += 360.0;
  }
  return angle;
}

float yawForHeading(int8_t heading)
{
  return gridHeadingZeroYawDeg + 90.0 * (float)((heading + 4) % 4);
}

void updateEncoderFromPins(uint8_t index)
{
  bool a = digitalRead(ENC_A[index]);
  bool b = digitalRead(ENC_B[index]);
  encoderCounts[index] += (a == b) ? 1 : -1;
}

void pollEncoders()
{
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    bool a = digitalRead(ENC_A[i]);
    if (a != lastEncoderA[i])
    {
      lastEncoderA[i] = a;
      updateEncoderFromPins(i);
    }
  }
}

void copyEncoderCounts(long target[ENCODER_COUNT])
{
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    target[i] = encoderCounts[i];
  }
}

long encoderMotionSince(const long previousCounts[ENCODER_COUNT])
{
  long motion = 0;
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    motion += labs(encoderCounts[i] - previousCounts[i]);
  }
  return motion;
}

void beginEncoders()
{
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    lastEncoderA[i] = digitalRead(ENC_A[i]);
    encoderCounts[i] = 0;
    nodeStartEncoderCounts[i] = 0;
    lastStatusEncoderCounts[i] = 0;
  }
}

bool initRFIDNoResetPin()
{
  Wire1.beginTransmission(RFID_I2C_ADDR);
  if (Wire1.endTransmission() != 0)
  {
    Serial.println("RFID | Not found at 0x28.");
    return false;
  }

  mfrc522.PCD_Reset();
  mfrc522.PCD_WriteRegister(MFRC522_I2C::TModeReg, 0x80);
  mfrc522.PCD_WriteRegister(MFRC522_I2C::TPrescalerReg, 0xA9);
  mfrc522.PCD_WriteRegister(MFRC522_I2C::TReloadRegH, 0x03);
  mfrc522.PCD_WriteRegister(MFRC522_I2C::TReloadRegL, 0xE8);
  mfrc522.PCD_WriteRegister(MFRC522_I2C::TxASKReg, 0x40);
  mfrc522.PCD_WriteRegister(MFRC522_I2C::ModeReg, 0x3D);
  mfrc522.PCD_AntennaOn();
  mfrc522.PCD_SetAntennaGain(MFRC522_I2C::RxGain_max);

  byte version = mfrc522.PCD_ReadRegister(MFRC522_I2C::VersionReg);
  Serial.print("RFID | Ready version=0x");
  if (version < 0x10)
  {
    Serial.print("0");
  }
  Serial.println(version, HEX);
  return version != 0x00 && version != 0xFF;
}

uint32_t hashRFIDUid()
{
  uint32_t hash = 2166136261UL;
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    hash ^= mfrc522.uid.uidByte[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool readRFIDCard(uint32_t &uidHash)
{
  byte atqa[2];
  byte atqaSize = sizeof(atqa);
  byte status = mfrc522.PICC_RequestA(atqa, &atqaSize);
  if (status != MFRC522_I2C::STATUS_OK && status != MFRC522_I2C::STATUS_COLLISION)
  {
    atqaSize = sizeof(atqa);
    status = mfrc522.PICC_WakeupA(atqa, &atqaSize);
  }

  if (status != MFRC522_I2C::STATUS_OK && status != MFRC522_I2C::STATUS_COLLISION)
  {
    return false;
  }

  if (!mfrc522.PICC_ReadCardSerial())
  {
    return false;
  }

  lastRFIDTagId[0] = '\0';
  size_t tagOffset = 0;
  for (byte i = 0; i < mfrc522.uid.size && tagOffset + 2 < sizeof(lastRFIDTagId); i++)
  {
    snprintf(lastRFIDTagId + tagOffset,
             sizeof(lastRFIDTagId) - tagOffset,
             "%02X",
             mfrc522.uid.uidByte[i]);
    tagOffset += 2;
  }

  uidHash = hashRFIDUid();
  mfrc522.PICC_HaltA();
  return true;
}

bool sendServerText(const char *payload)
{
  if (!USE_SERVER_COORDINATES || !serverMessengerStarted || payload == nullptr)
  {
    return false;
  }

  bool sent = serverMessenger.sendToBoard(SERVER_TARGET_BOARD, payload);
  Serial.print(sent ? "SERVER TX | " : "SERVER TX_FAIL | ");
  Serial.println(payload);
  return sent;
}

bool sendServerRegister()
{
  char payload[96];
  snprintf(payload, sizeof(payload),
           "type=register team_id=%s board_id=%s",
           GROUP_ID,
           BOARD_ID);
  return sendServerText(payload);
}

bool sendServerIsFertile(const char *tagId)
{
  char payload[144];
  snprintf(payload, sizeof(payload),
           "type=isFertile team_id=%s tag_id=%s board_id=%s",
           GROUP_ID,
           tagId == nullptr || tagId[0] == '\0' ? "UNKNOWN" : tagId,
           BOARD_ID);
  return sendServerText(payload);
}

bool inferHeadingFromServerDelta(int8_t previousX, int8_t previousY, int8_t currentX, int8_t currentY, int8_t &heading)
{
  int dx = currentX - previousX;
  int dy = currentY - previousY;
  if (dx == 0 && dy == 0)
  {
    return false;
  }

  if (dx != 0 && dy == 0)
  {
    heading = dx > 0 ? GRID_HEADING_EAST : GRID_HEADING_WEST;
    return true;
  }

  if (dy != 0 && dx == 0)
  {
    heading = dy > 0 ? GRID_HEADING_NORTH : GRID_HEADING_SOUTH;
    return true;
  }

  return false;
}

void calibrateHeadingFromServerDelta(int8_t previousX, int8_t previousY, int8_t currentX, int8_t currentY)
{
  int8_t inferredHeading = robotGridHeading;
  if (!inferHeadingFromServerDelta(previousX, previousY, currentX, currentY, inferredHeading))
  {
    Serial.print("RFID_HEADING | ambiguous/no movement prev=(");
    Serial.print(previousX);
    Serial.print(",");
    Serial.print(previousY);
    Serial.print(") now=(");
    Serial.print(currentX);
    Serial.print(",");
    Serial.print(currentY);
    Serial.println("), position only.");
    return;
  }

  robotGridHeading = inferredHeading;
  gridHeadingZeroYawDeg = yawDeg - 90.0 * (float)robotGridHeading;
  setTargetHeading(targetHeading);
  Serial.print("RFID_HEADING | prev=(");
  Serial.print(previousX);
  Serial.print(",");
  Serial.print(previousY);
  Serial.print(") now=(");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.print(") inferred=");
  Serial.print(headingName(robotGridHeading));
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" zeroYaw=");
  Serial.println(gridHeadingZeroYawDeg, 1);
}

bool serverHeadingCalibrationAllowed()
{
  if (returnTurnActive)
  {
    return false;
  }

  return state == RANDOM_FORWARD || state == DRIVE_X || state == DRIVE_Y;
}

void applyServerCoordinate(int x, int y, const char *source)
{
  int8_t newX = constrain((int8_t)x, ARENA_MIN_COORD, ARENA_MAX_COORD);
  int8_t newY = constrain((int8_t)y, ARENA_MIN_COORD, ARENA_MAX_COORD);
  if (haveLastServerCoordinate)
  {
    if (serverHeadingCalibrationAllowed())
    {
      calibrateHeadingFromServerDelta(lastServerX, lastServerY, newX, newY);
    }
    else
    {
      Serial.print("RFID_HEADING | calibration deferred while state=");
      Serial.print(stateName(state));
      Serial.println("; preserving IMU heading reference.");
    }
  }

  robotGridX = newX;
  robotGridY = newY;
  lastServerX = newX;
  lastServerY = newY;
  haveLastServerCoordinate = true;
  positionKnown = true;
  serverCoordinateKnown = true;
  waitingForFirstServerCoordinate = false;
  waitingForLegServerCoordinate = false;
  Serial.print("SERVER_POS | ");
  Serial.print(source);
  Serial.print(" -> (");
  Serial.print(robotGridX);
  Serial.print(",");
  Serial.print(robotGridY);
  Serial.println(")");
}

void maybePlanAfterServerCoordinate(const char *source)
{
  if (state == DONE || state == FAILED || state == TUNNEL_TO_BASE || state == BASE_RIGHT_PRIORITY)
  {
    Serial.print("SERVER_POS | ");
    Serial.print(source);
    Serial.println(" accepted, no route replan in current terminal/base state.");
    return;
  }

  if (state == OBSTACLE_AVOID)
  {
    routeReplanPending = true;
    Serial.print("SERVER_POS | ");
    Serial.print(source);
    Serial.println(" accepted, defer route replan until obstacle maneuver finishes.");
    return;
  }

  if (state == ALIGN_X || state == ALIGN_Y)
  {
    routeReplanPending = true;
    Serial.print("SERVER_POS | ");
    Serial.print(source);
    Serial.println(" accepted, defer route replan until current turn finishes.");
    return;
  }

  planNextLeg();
}

void onServerMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length)
{
  char message[MiniMessenger::kMaxPayloadSize + 1];
  size_t copyLen = length;
  if (copyLen > MiniMessenger::kMaxPayloadSize)
  {
    copyLen = MiniMessenger::kMaxPayloadSize;
  }
  memcpy(message, payload, copyLen);
  message[copyLen] = '\0';

  Serial.print("SERVER RX | from=");
  Serial.print(metadata.fromBoardId);
  Serial.print(" ");
  Serial.println(message);

  if (messageTypeEquals(message, "isFertileReply"))
  {
    int x = 0;
    int y = 0;
    bool haveX = parseIntField(message, "x", x);
    bool haveY = parseIntField(message, "y", y);
    if (haveX && haveY)
    {
      applyServerCoordinate(x, y, "isFertileReply");
      maybePlanAfterServerCoordinate("isFertileReply");
    }
    return;
  }

  if (messageTypeEquals(message, "position") || messageTypeEquals(message, "positionReply"))
  {
    int x = 0;
    int y = 0;
    if (parseIntField(message, "x", x) && parseIntField(message, "y", y))
    {
      applyServerCoordinate(x, y, "positionReply");
      maybePlanAfterServerCoordinate("positionReply");
    }
  }
}

void setupServerMessenger()
{
  if (!USE_SERVER_COORDINATES)
  {
    Serial.println("SERVER | Disabled for back_to_base.");
    return;
  }

  serverMessenger.onMessage(onServerMessage);
  serverMessengerStarted = true;
  bool connected = serverMessenger.begin(WIFI_SSID,
                                         WIFI_PASSWORD,
                                         BROKER_HOST,
                                         BROKER_PORT,
                                         GROUP_ID,
                                         BOARD_ID);

  Serial.print("SERVER | MiniMessenger ");
  Serial.print(connected ? "connected" : "connecting");
  Serial.print(" group=");
  Serial.print(GROUP_ID);
  Serial.print(" board=");
  Serial.println(BOARD_ID);
}

void updateServerMessenger()
{
  if (!USE_SERVER_COORDINATES || !serverMessengerStarted)
  {
    return;
  }

  serverMessenger.loop();
  bool connected = serverMessenger.isConnected();
  if (connected != serverLastConnected)
  {
    serverLastConnected = connected;
    Serial.print("SERVER | ");
    Serial.println(connected ? "connected" : "disconnected");
    if (connected)
    {
      serverLastRegisterMs = 0;
    }
  }

  if (connected && (serverLastRegisterMs == 0 || millis() - serverLastRegisterMs >= SERVER_REGISTER_INTERVAL_MS))
  {
    if (sendServerRegister())
    {
      serverLastRegisterMs = millis();
    }
  }
}

bool acceptUniqueRFID(uint32_t uidHash)
{
  unsigned long now = millis();
  if (haveLastNodeUid && uidHash == lastNodeUid && now - lastNodeUidMs < NODE_REPEAT_MS)
  {
    return false;
  }

  haveLastNodeUid = true;
  lastNodeUid = uidHash;
  lastNodeUidMs = now;
  return true;
}

void printPosition(const char *source)
{
  Serial.print("ARENA | ");
  Serial.print(source);
  Serial.print(" pos=(");
  Serial.print(robotGridX);
  Serial.print(",");
  Serial.print(robotGridY);
  Serial.print(") heading=");
  Serial.print(headingName(robotGridHeading));
  Serial.print(" known=");
  Serial.println(positionKnown ? "YES" : "NO");
}

void fail(const char *reason);

void advancePositionOneNode(const char *source)
{
  if (!positionKnown)
  {
    Serial.print("ARENA | ");
    Serial.print(source);
    Serial.println(" node passed, but position is unknown until server x/y.");
    return;
  }

  switch (robotGridHeading)
  {
    case GRID_HEADING_NORTH: robotGridY++; break;
    case GRID_HEADING_EAST: robotGridX++; break;
    case GRID_HEADING_SOUTH: robotGridY--; break;
    case GRID_HEADING_WEST: robotGridX--; break;
  }

  robotGridX = constrain(robotGridX, ARENA_MIN_COORD, ARENA_MAX_COORD);
  robotGridY = constrain(robotGridY, ARENA_MIN_COORD, ARENA_MAX_COORD);
  printPosition(source);
}

bool countNodeByRFID(const char *source)
{
  unsigned long now = millis();
  if (!rfidReady || now - lastRFIDCheckMs < RFID_CHECK_INTERVAL_MS)
  {
    return false;
  }
  lastRFIDCheckMs = now;

  uint32_t uidHash = 0;
  if (!readRFIDCard(uidHash))
  {
    return false;
  }

  if (state == RANDOM_FORWARD && randomNodesCounted == 0 && now - stateStartMs < START_RFID_IGNORE_MS)
  {
    haveLastNodeUid = true;
    lastNodeUid = uidHash;
    lastNodeUidMs = now;
    Serial.print("NODE | Start-card ignored uid=0x");
    Serial.println(uidHash, HEX);
    return false;
  }

  if (!acceptUniqueRFID(uidHash))
  {
    return false;
  }

  obstacleDepth = 0;
  printWallSnapshot(source);

  bool firstLocalizationTag = state == RANDOM_FORWARD && !serverCoordinateKnown;
  if (firstLocalizationTag)
  {
    positionKnown = false;
    Serial.println("NODE | First localization RFID. Waiting for server x/y before planning.");
  }
  else
  {
    advancePositionOneNode(source);
  }

  if (USE_FIRST_TAG_COORD_FALLBACK && !serverCoordinateKnown && !firstTagFallbackUsed)
  {
    firstTagFallbackUsed = true;
    applyServerCoordinate(FIRST_SERVER_TAG_X, FIRST_SERVER_TAG_Y, "first-tag fallback");
  }
  if (USE_SERVER_COORDINATES &&
      serverMessengerStarted &&
      serverMessenger.isConnected() &&
      millis() - lastServerRfidRequestMs >= SERVER_REQUEST_COOLDOWN_MS)
  {
    if (sendServerIsFertile(lastRFIDTagId))
    {
      lastServerRfidRequestMs = millis();
    }
  }
  nodesTravelled++;
  if (state == RANDOM_FORWARD)
  {
    randomNodesCounted++;
  }
  nodeStartMs = millis();
  copyEncoderCounts(nodeStartEncoderCounts);

  Serial.print("NODE | RFID travelled=");
  Serial.print(nodesTravelled);
  Serial.print("/");
  Serial.print(targetNodes);
  Serial.print(" uid=0x");
  Serial.println(uidHash, HEX);
  return true;
}

bool checkNodeRFIDTimeout(const char *source)
{
  if (millis() - nodeStartMs < NODE_TIMEOUT_MS)
  {
    return false;
  }

  Serial.print("NODE | RFID timeout while waiting for node ");
  Serial.print(nodesTravelled + 1);
  Serial.print("/");
  Serial.print(targetNodes);
  Serial.print(" source=");
  Serial.println(source);
  fail("RFID node timeout.");
  return true;
}

bool countNodeByEncoder(const char *source)
{
  long motion = encoderMotionSince(nodeStartEncoderCounts);
  if (motion < NODE_ENCODER_TICKS)
  {
    return false;
  }

  advancePositionOneNode(source);
  nodesTravelled++;
  if (state == RANDOM_FORWARD)
  {
    randomNodesCounted++;
  }
  nodeStartMs = millis();
  copyEncoderCounts(nodeStartEncoderCounts);

  Serial.print("NODE | encoder travelled=");
  Serial.print(nodesTravelled);
  Serial.print("/");
  Serial.print(targetNodes);
  Serial.print(" motion=");
  Serial.println(motion);
  return true;
}

bool countNodeByTimeout(const char *source)
{
  if (millis() - nodeStartMs < NODE_TIMEOUT_MS)
  {
    return false;
  }

  advancePositionOneNode(source);
  nodesTravelled++;
  if (state == RANDOM_FORWARD)
  {
    randomNodesCounted++;
  }
  nodeStartMs = millis();
  copyEncoderCounts(nodeStartEncoderCounts);

  Serial.print("NODE | timeout travelled=");
  Serial.print(nodesTravelled);
  Serial.print("/");
  Serial.println(targetNodes);
  return true;
}

void updateRFIDReady()
{
  if (rfidReady)
  {
    return;
  }

  if (millis() - lastRFIDRetryMs >= RFID_RETRY_INTERVAL_MS)
  {
    lastRFIDRetryMs = millis();
    rfidReady = initRFIDNoResetPin();
  }
}

void enterState(BackState nextState)
{
  if (nextState != ALIGN_X && nextState != ALIGN_Y)
  {
    returnTurnActive = false;
  }
  state = nextState;
  stateStartMs = millis();
  turnSettleStartMs = 0;
  Serial.print("STATE | ");
  Serial.println(stateName(state));
}

void fail(const char *reason)
{
  stopAllMotors();
  Serial.print("FAIL | ");
  Serial.println(reason);
  enterState(FAILED);
}

void finish()
{
  stopAllMotors();
  Serial.print("RETURN | Reached exit tag target (");
  Serial.print(ARENA_EXIT_X);
  Serial.print(",");
  Serial.print(ARENA_EXIT_Y);
  Serial.println("). Enter tunnel-to-base mode.");
  wallHasLast = false;
  enterState(TUNNEL_TO_BASE);
}

void beginNodeDrive(BackState nextState, uint8_t nodeTarget)
{
  nodesTravelled = 0;
  targetNodes = nodeTarget;
  nodeStartMs = millis();
  copyEncoderCounts(nodeStartEncoderCounts);
  copyEncoderCounts(lastStatusEncoderCounts);
  resetLinePid();
  lastDriveUsedLine = false;
  enterState(nextState);
}

void setTargetHeading(int8_t heading)
{
  targetHeading = (heading + 4) % 4;
  targetYawDeg = yawForHeading(targetHeading);
}

bool headingWouldLeaveArena(int8_t heading)
{
  return (heading == GRID_HEADING_NORTH && robotGridY >= ARENA_MAX_COORD) ||
         (heading == GRID_HEADING_EAST && robotGridX >= ARENA_MAX_COORD) ||
         (heading == GRID_HEADING_SOUTH && robotGridY <= ARENA_MIN_COORD) ||
         (heading == GRID_HEADING_WEST && robotGridX <= ARENA_MIN_COORD);
}

void planNextLeg()
{
  routeReplanPending = false;
  waitingForLegServerCoordinate = false;

  if (!positionKnown || (USE_SERVER_COORDINATES && !serverCoordinateKnown))
  {
    stopAllMotors();
    Serial.println("RETURN | Waiting for server coordinate before route planning.");
    if (waitingForFirstServerCoordinate)
    {
      return;
    }
    fail("No server coordinate for return route.");
    return;
  }

  if (robotGridX == ARENA_EXIT_X && robotGridY == ARENA_EXIT_Y)
  {
    finish();
    return;
  }

  printWallSnapshot("PLAN");

  int dy = ARENA_EXIT_Y - robotGridY;
  int dx = ARENA_EXIT_X - robotGridX;
  Serial.print("ROUTE | from=(");
  Serial.print(robotGridX);
  Serial.print(",");
  Serial.print(robotGridY);
  Serial.print(") exit=(");
  Serial.print(ARENA_EXIT_X);
  Serial.print(",");
  Serial.print(ARENA_EXIT_Y);
  Serial.print(") dx=");
  Serial.print(dx);
  Serial.print(" dy=");
  Serial.println(dy);

  int8_t yHeading = dy > 0 ? GRID_HEADING_NORTH : GRID_HEADING_SOUTH;
  int8_t xHeading = dx > 0 ? GRID_HEADING_EAST : GRID_HEADING_WEST;
  bool hasY = dy != 0;
  bool hasX = dx != 0;
  bool chooseY = hasY;

  if (hasY && hasX && headingBlockedByWall(yHeading) && !headingBlockedByWall(xHeading))
  {
    chooseY = false;
    Serial.println("PLAN | Preferred Y leg blocked by wall; trying X leg first.");
  }
  else if (hasY && headingBlockedByWall(yHeading))
  {
    Serial.println("PLAN | Preferred Y leg looks blocked; no open X alternative, will rely on obstacle handling.");
  }
  else if (!hasY && hasX && headingBlockedByWall(xHeading))
  {
    Serial.println("PLAN | X leg looks blocked; will rely on obstacle handling.");
  }

  if (chooseY && hasY)
  {
    setTargetHeading(yHeading);
    targetNodes = (uint8_t)abs(dy);
    nodesTravelled = 0;
    copyEncoderCounts(nodeStartEncoderCounts);
    nodeStartMs = millis();
    Serial.print("PLAN | leg=Y heading=");
    Serial.print(headingName(targetHeading));
    Serial.print(" nodes=");
    Serial.println(targetNodes);
    enterState(ALIGN_Y);
    return;
  }

  if (dx != 0)
  {
    setTargetHeading(xHeading);
    targetNodes = (uint8_t)abs(dx);
    nodesTravelled = 0;
    copyEncoderCounts(nodeStartEncoderCounts);
    nodeStartMs = millis();
    Serial.print("PLAN | leg=X heading=");
    Serial.print(headingName(targetHeading));
    Serial.print(" nodes=");
    Serial.println(targetNodes);
    enterState(ALIGN_X);
    return;
  }

  finish();
}

void driveYawHold(int16_t baseSpeed)
{
  float yawError = normalizeAngleDeg(targetYawDeg - yawDeg);
  int16_t correction = constrain(
      (int16_t)(yawError * RETURN_YAW_KP),
      (int16_t)-RETURN_MAX_CORRECTION,
      (int16_t) RETURN_MAX_CORRECTION);
  int16_t leftSpeed = constrain((int16_t)(baseSpeed + correction), -MAX_SPEED, MAX_SPEED);
  int16_t rightSpeed = constrain((int16_t)(baseSpeed - correction), -MAX_SPEED, MAX_SPEED);
  driveBySide(leftSpeed, rightSpeed);
}

bool driveWallFollow(int16_t baseSpeed, int16_t minimumSpeed)
{
  float leftCm = leftDistanceCm();
  float rightCm = rightDistanceCm();
  bool leftValid = validWallDistance(leftCm);
  bool rightValid = validWallDistance(rightCm);
  if (!leftValid && !rightValid)
  {
    wallHasLast = false;
    return false;
  }

  float error = 0.0;
  if (leftValid && rightValid)
  {
    error = (rightCm - leftCm) * 0.5;
  }
  else if (leftValid)
  {
    error = WALL_TARGET_CM - leftCm;
  }
  else
  {
    error = rightCm - WALL_TARGET_CM;
  }

  unsigned long now = millis();
  float dtSec = wallHasLast && now > wallLastMs ? (now - wallLastMs) / 1000.0 : 0.02;
  if (dtSec < 0.01)
  {
    dtSec = 0.01;
  }
  float derivative = wallHasLast ? (error - wallLastError) / dtSec : 0.0;
  int16_t correction = constrain((int16_t)(WALL_KP * error + WALL_KD * derivative),
                                 (int16_t)-WALL_MAX_CORRECTION,
                                 (int16_t) WALL_MAX_CORRECTION);
  int16_t speed = (leftCm > 0.0 && leftCm < WALL_CRITICAL_CM) ||
                          (rightCm > 0.0 && rightCm < WALL_CRITICAL_CM)
                      ? minimumSpeed
                      : baseSpeed;

  driveBySide(speed + correction, speed - correction);
  wallLastError = error;
  wallLastMs = now;
  wallHasLast = true;
  return true;
}

bool inEntryCorridor()
{
  return state == RANDOM_FORWARD &&
         robotGridX == ARENA_ENTRY_X &&
         robotGridY == ARENA_ENTRY_Y;
}

void printEntryGuideStatus(const char *mode, float frontCm, float leftCm, float rightCm)
{
  if (millis() - lastEntryGuidePrintMs < 350)
  {
    return;
  }

  lastEntryGuidePrintMs = millis();
  Serial.print("ENTRY_GUIDE | mode=");
  Serial.print(mode);
  Serial.print(" front=");
  Serial.print(frontCm, 1);
  Serial.print(" left=");
  Serial.print(leftCm, 1);
  Serial.print(" right=");
  Serial.println(rightCm, 1);
}

void driveEntryWallFollow(float leftCm, float rightCm)
{
  bool leftValid = validWallDistance(leftCm);
  bool rightValid = validWallDistance(rightCm);
  float error = 0.0;

  if (leftValid && rightValid)
  {
    error = (rightCm - leftCm) * 0.5;
  }
  else if (leftValid && leftCm < ENTRY_CORRIDOR_SINGLE_WALL_CM)
  {
    error = ENTRY_CORRIDOR_SINGLE_WALL_CM - leftCm;
  }
  else if (rightValid && rightCm < ENTRY_CORRIDOR_SINGLE_WALL_CM)
  {
    error = rightCm - ENTRY_CORRIDOR_SINGLE_WALL_CM;
  }
  else
  {
    wallHasLast = false;
    driveYawHold(ENTRY_CORRIDOR_MIN_SPEED);
    return;
  }

  unsigned long now = millis();
  float dtSec = wallHasLast && now > wallLastMs ? (now - wallLastMs) / 1000.0 : 0.02;
  if (dtSec < 0.01)
  {
    dtSec = 0.01;
  }
  float derivative = wallHasLast ? (error - wallLastError) / dtSec : 0.0;
  int16_t correction = constrain((int16_t)(ENTRY_CORRIDOR_WALL_KP * error +
                                           ENTRY_CORRIDOR_WALL_KD * derivative),
                                 (int16_t)-ENTRY_CORRIDOR_MAX_CORRECTION,
                                 (int16_t) ENTRY_CORRIDOR_MAX_CORRECTION);
  driveBySide(ENTRY_CORRIDOR_WALL_SPEED + correction,
              ENTRY_CORRIDOR_WALL_SPEED - correction);

  wallLastError = error;
  wallLastMs = now;
  wallHasLast = true;
}

void driveEntryCorridorGuide()
{
  readSensors();
  float lineError = 0.0;
  if (computeLineError(lineError))
  {
    if (entryCorridorUsingWall)
    {
      Serial.println("ENTRY_GUIDE | Line restored. Slow line PID.");
    }
    entryCorridorUsingWall = false;
    entryCorridorEscapeUntilMs = 0;
    wallHasLast = false;
    lastDriveUsedLine = true;
    driveLinePid(lineError, ENTRY_CORRIDOR_LINE_SPEED);
    return;
  }

  lastDriveUsedLine = false;
  resetLinePid();

  float frontCm = frontDistanceCm();
  float leftCm = leftDistanceCm();
  float rightCm = rightDistanceCm();
  unsigned long now = millis();

  if (now < entryCorridorEscapeUntilMs)
  {
    pivotDirection(entryCorridorEscapeDirection, ENTRY_CORRIDOR_ESCAPE_SPEED);
    printEntryGuideStatus(entryCorridorEscapeDirection > 0 ? "ESCAPE_RIGHT" : "ESCAPE_LEFT",
                          frontCm, leftCm, rightCm);
    entryCorridorUsingWall = true;
    return;
  }

  bool frontBlocked = frontCm > 0.0 && frontCm <= ENTRY_CORRIDOR_FRONT_BLOCK_CM;
  bool leftCritical = leftCm > 0.0 && leftCm <= ENTRY_CORRIDOR_CRITICAL_CM;
  bool rightCritical = rightCm > 0.0 && rightCm <= ENTRY_CORRIDOR_CRITICAL_CM;
  if (frontBlocked || leftCritical || rightCritical)
  {
    if (leftCritical && !rightCritical)
    {
      entryCorridorEscapeDirection = 1;
    }
    else if (rightCritical && !leftCritical)
    {
      entryCorridorEscapeDirection = -1;
    }
    else
    {
      entryCorridorEscapeDirection = rightCm > leftCm ? 1 : -1;
    }
    entryCorridorEscapeUntilMs = now + ENTRY_CORRIDOR_ESCAPE_MS;
    wallHasLast = false;
    pivotDirection(entryCorridorEscapeDirection, ENTRY_CORRIDOR_ESCAPE_SPEED);
    Serial.print("ENTRY_GUIDE | Too close to wall. Short escape ");
    Serial.println(entryCorridorEscapeDirection > 0 ? "RIGHT." : "LEFT.");
    return;
  }

  if (!entryCorridorUsingWall)
  {
    Serial.println("ENTRY_GUIDE | Line lost at (9,3). Conservative wall following enabled.");
  }
  entryCorridorUsingWall = true;
  driveEntryWallFollow(leftCm, rightCm);
  printEntryGuideStatus("WALL_SLOW", frontCm, leftCm, rightCm);
}

const char *returnTurnPhaseName(ReturnTurnPhase phase)
{
  switch (phase)
  {
    case RETURN_TURN_APPROACH: return "APPROACH";
    case RETURN_TURN_SEGMENT_STOP: return "SEGMENT_STOP";
    case RETURN_TURN_RELIEF_FORWARD: return "RELIEF_FORWARD";
    case RETURN_TURN_RELIEF_BACKWARD: return "RELIEF_BACKWARD";
    case RETURN_TURN_SETTLE: return "SETTLE";
    case RETURN_TURN_FINE_CORRECT: return "FINE_CORRECT";
  }
  return "UNKNOWN";
}

float returnTurnErrorDeg()
{
  return normalizeAngleDeg(targetYawDeg - yawDeg);
}

float returnTurnProgressDeg()
{
  return fabs(normalizeAngleDeg(yawDeg - returnTurnStartYawDeg));
}

float limitedReturnTurnSegmentTarget(float currentDeg)
{
  float target = currentDeg + RETURN_TURN_SEGMENT_DEG;
  if (target > returnTurnTargetDeltaDeg)
  {
    target = returnTurnTargetDeltaDeg;
  }
  return target;
}

int16_t returnTurnSpeedForError(float errorAbs)
{
  if (errorAbs > RETURN_TURN_MID_REMAINING_DEG)
  {
    return RETURN_TURN_FAST_SPEED;
  }

  if (errorAbs > RETURN_TURN_FINE_REMAINING_DEG)
  {
    return RETURN_TURN_MID_SPEED;
  }

  return RETURN_TURN_FINE_SPEED;
}

void enterReturnTurnPhase(ReturnTurnPhase nextPhase)
{
  returnTurnPhase = nextPhase;
  returnTurnPhaseStartMs = millis();
  Serial.print("RETURN_TURN_PHASE | ");
  Serial.println(returnTurnPhaseName(returnTurnPhase));
}

void beginReturnTurn()
{
  returnTurnStartYawDeg = yawDeg;
  returnTurnTargetDeltaDeg = fabs(normalizeAngleDeg(targetYawDeg - returnTurnStartYawDeg));
  returnTurnSegmentTargetDeg = limitedReturnTurnSegmentTarget(0.0);
  returnTurnNextReliefForward = true;
  returnTurnActive = true;
  lastReturnTurnPrintMs = 0;
  enterReturnTurnPhase(RETURN_TURN_APPROACH);

  Serial.print("RETURN_TURN | targetHeading=");
  Serial.print(headingName(targetHeading));
  Serial.print(" startYaw=");
  Serial.print(returnTurnStartYawDeg, 1);
  Serial.print(" targetYaw=");
  Serial.print(targetYawDeg, 1);
  Serial.print(" delta=");
  Serial.println(returnTurnTargetDeltaDeg, 1);
}

void printReturnTurnStatus(int16_t speedCommand)
{
  if (millis() - lastReturnTurnPrintMs < STATUS_INTERVAL_MS)
  {
    return;
  }

  lastReturnTurnPrintMs = millis();
  Serial.print("RETURN_TURN | phase=");
  Serial.print(returnTurnPhaseName(returnTurnPhase));
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" progress=");
  Serial.print(returnTurnProgressDeg(), 1);
  Serial.print("/");
  Serial.print(returnTurnTargetDeltaDeg, 1);
  Serial.print(" error=");
  Serial.print(returnTurnErrorDeg(), 1);
  Serial.print(" speed=");
  Serial.print(speedCommand);
  Serial.print(" segmentTarget=");
  Serial.println(returnTurnSegmentTargetDeg, 1);
}

bool blackLineAvailableHere()
{
  return robotGridY <= 4;
}

void driveByBestAvailableGuide(int16_t noLineBaseSpeed)
{
  readSensors();
  float lineError = 0.0;
  if (computeLineError(lineError))
  {
    if (!lastDriveUsedLine)
    {
      resetLinePid();
      Serial.println("GUIDE | Black line detected. Using line PID regardless of coordinate.");
    }
    lastDriveUsedLine = true;
    driveLinePid(lineError);
    return;
  }

  if (lastDriveUsedLine)
  {
    Serial.println("GUIDE | Line lost. Holding planned IMU heading.");
  }
  lastDriveUsedLine = false;
  resetLinePid();
  driveYawHold(blackLineAvailableHere() ? LINE_LOST_YAW_SPEED : noLineBaseSpeed);
}

void randomForwardStep()
{
  if (waitingForFirstServerCoordinate)
  {
    stopAllMotors();
    if (serverCoordinateKnown)
    {
      waitingForFirstServerCoordinate = false;
      planNextLeg();
      return;
    }

    if (USE_SERVER_COORDINATES &&
        serverMessengerStarted &&
        serverMessenger.isConnected() &&
        millis() - lastServerRfidRequestMs >= SERVER_REQUEST_COOLDOWN_MS)
    {
      if (sendServerIsFertile(lastRFIDTagId))
      {
        lastServerRfidRequestMs = millis();
      }
    }

    if (millis() - firstServerCoordWaitStartMs >= FIRST_TAG_SERVER_WAIT_MS)
    {
      waitingForFirstServerCoordinate = false;
      Serial.println("START | Server x/y wait timeout after first RFID.");
      planNextLeg();
      return;
    }

    return;
  }

  setTargetHeading(GRID_HEADING_NORTH);
  bool counted = countNodeByRFID("START_RFID");

  if (counted && !serverCoordinateKnown && USE_SERVER_COORDINATES)
  {
    stopAllMotors();
    waitingForFirstServerCoordinate = true;
    firstServerCoordWaitStartMs = millis();
    Serial.println("START | First RFID seen. Stopping briefly for server coordinate.");
    return;
  }

  if (counted && randomNodesCounted >= START_FORWARD_NODES)
  {
    stopAllMotors();
    Serial.print("START | Complete. RFID nodes=");
    Serial.println(randomNodesCounted);
    planNextLeg();
    return;
  }

  if (inEntryCorridor())
  {
    driveEntryCorridorGuide();
    checkNodeRFIDTimeout("START_ENTRY_CORRIDOR");
    return;
  }

  if (frontObstacleDetected())
  {
    startObstacleAvoidance("start-forward");
    return;
  }
  driveByBestAvailableGuide(RANDOM_FORWARD_SPEED);

  checkNodeRFIDTimeout("START_FORWARD");
}

void alignStep(BackState driveState)
{
  if (!returnTurnActive)
  {
    beginReturnTurn();
  }

  unsigned long now = millis();
  if (now - stateStartMs >= RETURN_TURN_TIMEOUT_MS)
  {
    fail("Segmented turn timeout.");
    return;
  }

  float yawError = returnTurnErrorDeg();
  float errorAbs = fabs(yawError);
  float progressDeg = returnTurnProgressDeg();
  int16_t speedCommand = 0;

  switch (returnTurnPhase)
  {
    case RETURN_TURN_APPROACH:
      if (errorAbs <= RETURN_TURN_TOLERANCE_DEG)
      {
        stopAllMotors();
        enterReturnTurnPhase(RETURN_TURN_SETTLE);
      }
      else if (progressDeg >= returnTurnSegmentTargetDeg && errorAbs > RETURN_TURN_FINE_REMAINING_DEG)
      {
        stopAllMotors();
        enterReturnTurnPhase(RETURN_TURN_SEGMENT_STOP);
      }
      else
      {
        speedCommand = returnTurnSpeedForError(errorAbs);
        pivotDirection(yawError > 0.0 ? 1 : -1, speedCommand);
      }
      break;

    case RETURN_TURN_SEGMENT_STOP:
      stopAllMotors();
      if (now - returnTurnPhaseStartMs >= RETURN_TURN_SEGMENT_STOP_MS)
      {
        yawError = returnTurnErrorDeg();
        errorAbs = fabs(yawError);
        if (errorAbs <= RETURN_TURN_TOLERANCE_DEG)
        {
          enterReturnTurnPhase(RETURN_TURN_SETTLE);
        }
        else if (errorAbs <= RETURN_TURN_FINE_REMAINING_DEG)
        {
          returnTurnSegmentTargetDeg = limitedReturnTurnSegmentTarget(returnTurnProgressDeg());
          enterReturnTurnPhase(RETURN_TURN_APPROACH);
        }
        else if (returnTurnNextReliefForward)
        {
          enterReturnTurnPhase(RETURN_TURN_RELIEF_FORWARD);
        }
        else
        {
          enterReturnTurnPhase(RETURN_TURN_RELIEF_BACKWARD);
        }
      }
      break;

    case RETURN_TURN_RELIEF_FORWARD:
      speedCommand = RETURN_TURN_RELIEF_SPEED;
      driveForward(RETURN_TURN_RELIEF_SPEED);
      if (now - returnTurnPhaseStartMs >= RETURN_TURN_RELIEF_PULSE_MS)
      {
        stopAllMotors();
        returnTurnNextReliefForward = false;
        returnTurnSegmentTargetDeg = limitedReturnTurnSegmentTarget(returnTurnProgressDeg());
        enterReturnTurnPhase(RETURN_TURN_APPROACH);
      }
      break;

    case RETURN_TURN_RELIEF_BACKWARD:
      speedCommand = -RETURN_TURN_RELIEF_SPEED;
      driveForward(-RETURN_TURN_RELIEF_SPEED);
      if (now - returnTurnPhaseStartMs >= RETURN_TURN_RELIEF_PULSE_MS)
      {
        stopAllMotors();
        returnTurnNextReliefForward = true;
        returnTurnSegmentTargetDeg = limitedReturnTurnSegmentTarget(returnTurnProgressDeg());
        enterReturnTurnPhase(RETURN_TURN_APPROACH);
      }
      break;

    case RETURN_TURN_SETTLE:
      stopAllMotors();
      if (now - returnTurnPhaseStartMs >= RETURN_TURN_SETTLE_MS)
      {
        yawError = returnTurnErrorDeg();
        errorAbs = fabs(yawError);
        if (errorAbs <= RETURN_TURN_TOLERANCE_DEG)
        {
          returnTurnActive = false;
          robotGridHeading = targetHeading;
          printPosition("HEADING_LOCK");
          if (routeReplanPending)
          {
            Serial.println("RETURN | Current turn complete. Replanning from deferred server coordinate.");
            planNextLeg();
          }
          else
          {
            beginNodeDrive(driveState, targetNodes);
          }
        }
        else
        {
          enterReturnTurnPhase(RETURN_TURN_FINE_CORRECT);
        }
      }
      break;

    case RETURN_TURN_FINE_CORRECT:
      yawError = returnTurnErrorDeg();
      errorAbs = fabs(yawError);
      if (errorAbs <= RETURN_TURN_TOLERANCE_DEG)
      {
        stopAllMotors();
        enterReturnTurnPhase(RETURN_TURN_SETTLE);
      }
      else
      {
        speedCommand = RETURN_TURN_FINE_SPEED;
        pivotDirection(yawError > 0.0 ? 1 : -1, speedCommand);
      }
      break;
  }

  printReturnTurnStatus(speedCommand);
}

void driveNodesStep()
{
  if (waitingForLegServerCoordinate)
  {
    stopAllMotors();
    if (millis() - legServerCoordWaitStartMs >= LEG_SERVER_COORD_WAIT_MS)
    {
      waitingForLegServerCoordinate = false;
      Serial.println("RETURN | Server coordinate wait expired. Planning from local node estimate.");
      planNextLeg();
    }
    return;
  }

  if (targetNodes == 0 || nodesTravelled >= targetNodes)
  {
    stopAllMotors();
    planNextLeg();
    return;
  }

  if (headingWouldLeaveArena(robotGridHeading))
  {
    stopAllMotors();
    Serial.print("ROUTE_GUARD | Refusing to leave arena while heading ");
    Serial.print(headingName(robotGridHeading));
    Serial.println(". Replanning.");
    planNextLeg();
    return;
  }

  if (frontObstacleDetected())
  {
    startObstacleAvoidance("return-leg");
    return;
  }

  bool counted = countNodeByRFID("RETURN_RFID");
  if (counted && nodesTravelled >= targetNodes)
  {
    stopAllMotors();
    if (USE_SERVER_COORDINATES && serverMessengerStarted && serverMessenger.isConnected())
    {
      waitingForLegServerCoordinate = true;
      legServerCoordWaitStartMs = millis();
      Serial.println("RETURN | Leg RFID reached. Holding heading for server coordinate before next turn.");
    }
    else
    {
      planNextLeg();
    }
    return;
  }

  if (!counted && !blackLineAvailableHere() && countNodeByEncoder("RETURN_ENCODER"))
  {
    if (nodesTravelled >= targetNodes)
    {
      stopAllMotors();
      planNextLeg();
      return;
    }
  }

  if (!counted && checkNodeRFIDTimeout("RETURN"))
  {
    return;
  }

  driveByBestAvailableGuide(RETURN_SPEED);
}

int8_t headingAfterTurn(int8_t heading, int direction)
{
  int8_t delta = direction > 0 ? 1 : -1;
  return (heading + delta + 4) % 4;
}

void enterObstaclePhase(ObstaclePhase phase)
{
  obstaclePhase = phase;
  obstaclePhaseStartMs = millis();
  Serial.print("OBS | phase=");
  Serial.print(obstaclePhaseName(obstaclePhase));
  Serial.print(" depth=");
  Serial.println(obstacleDepth);
}

void beginObstacleTurn(ObstaclePhase phase, int direction)
{
  obstacleTurnDirection = direction >= 0 ? 1 : -1;
  obstacleTurnStartYawDeg = yawDeg;
  enterObstaclePhase(phase);
}

void beginObstacleStraight(ObstaclePhase phase, uint8_t nodeTarget)
{
  obstacleNodeTarget = nodeTarget;
  obstacleNodesTravelled = 0;
  copyEncoderCounts(obstacleNodeStartEncoderCounts);
  nodeStartMs = millis();
  enterObstaclePhase(phase);
}

void startObstacleAvoidance(const char *source)
{
  if (obstacleDepth >= MAX_OBSTACLE_RECURSION_DEPTH)
  {
    fail("Obstacle recursion depth exceeded.");
    return;
  }

  obstacleDepth++;
  obstacleResumeState = state;
  float leftCm = leftDistanceCm();
  float rightCm = rightDistanceCm();
  int side = 1;
  if (validWallDistance(leftCm) || validWallDistance(rightCm))
  {
    side = rightCm >= leftCm ? 1 : -1;
  }

  float frontCm = frontDistanceCm();
  Serial.print("OBS | start source=");
  Serial.print(source);
  Serial.print(" front=");
  Serial.print(frontCm, 1);
  Serial.print(" left=");
  Serial.print(leftCm, 1);
  Serial.print(" right=");
  Serial.print(rightCm, 1);
  Serial.print(" side=");
  Serial.println(side > 0 ? "RIGHT" : "LEFT");

  obstacleTurnDirection = side;
  if (frontCm > 0.0 && frontCm <= FRONT_PINNED_CM)
  {
    Serial.println("OBS | Robot is pinned. Backing away before choosing a side.");
    enterObstaclePhase(OBS_BACK_AWAY);
  }
  else
  {
    beginObstacleTurn(OBS_TURN_OUT, side);
  }
  enterState(OBSTACLE_AVOID);
}

bool obstacleCountRFID()
{
  if (!rfidReady || millis() - lastRFIDCheckMs < RFID_CHECK_INTERVAL_MS)
  {
    return false;
  }
  lastRFIDCheckMs = millis();

  uint32_t uidHash = 0;
  if (!readRFIDCard(uidHash) || !acceptUniqueRFID(uidHash))
  {
    return false;
  }

  obstacleDepth = 0;
  printWallSnapshot("OBS_RFID");
  Serial.println("OBS_RFID | Detour RFID seen. Waiting for server coordinate; local detour does not change route position.");
  if (USE_SERVER_COORDINATES &&
      serverMessengerStarted &&
      serverMessenger.isConnected() &&
      millis() - lastServerRfidRequestMs >= SERVER_REQUEST_COOLDOWN_MS)
  {
    if (sendServerIsFertile(lastRFIDTagId))
    {
      lastServerRfidRequestMs = millis();
    }
  }

  obstacleNodesTravelled++;
  copyEncoderCounts(obstacleNodeStartEncoderCounts);
  nodeStartMs = millis();

  Serial.print("OBS_NODE | RFID ");
  Serial.print(obstacleNodesTravelled);
  Serial.print("/");
  Serial.println(obstacleNodeTarget);
  return true;
}

void obstacleCountEncoderFallback()
{
  long motion = encoderMotionSince(obstacleNodeStartEncoderCounts);
  if (motion < NODE_ENCODER_TICKS)
  {
    return;
  }

  obstacleNodesTravelled++;
  copyEncoderCounts(obstacleNodeStartEncoderCounts);
  nodeStartMs = millis();

  Serial.print("OBS_NODE | encoder ");
  Serial.print(obstacleNodesTravelled);
  Serial.print("/");
  Serial.print(obstacleNodeTarget);
  Serial.print(" motion=");
  Serial.println(motion);
}

void finishObstacleAvoidance()
{
  if (obstacleDepth > 0)
  {
    obstacleDepth--;
  }
  obstaclePhase = OBS_IDLE;
  stopAllMotors();
  Serial.println("OBS | finished. Replanning to base from current coordinate.");
  planNextLeg();
}

void obstacleTurnStep()
{
  float turnedDeg = fabs(normalizeAngleDeg(yawDeg - obstacleTurnStartYawDeg));
  float error = 90.0 - turnedDeg;
  if (fabs(error) <= RETURN_TURN_TOLERANCE_DEG)
  {
    stopAllMotors();
    robotGridHeading = headingAfterTurn(robotGridHeading, obstacleTurnDirection);
    setTargetHeading(robotGridHeading);
    Serial.print("OBS_TURN | done heading=");
    Serial.println(headingName(robotGridHeading));

    switch (obstaclePhase)
    {
      case OBS_TURN_OUT:
        beginObstacleStraight(OBS_SIDE_STEP_OUT, OBSTACLE_SIDE_STEP_NODES);
        break;
      case OBS_TURN_FORWARD:
        beginObstacleStraight(OBS_PASS_OBSTACLE, OBSTACLE_FORWARD_NODES);
        break;
      case OBS_TURN_BACK:
        beginObstacleStraight(OBS_SIDE_STEP_BACK, OBSTACLE_SIDE_STEP_NODES);
        break;
      case OBS_TURN_ORIGINAL:
        finishObstacleAvoidance();
        break;
      default:
        finishObstacleAvoidance();
        break;
    }
    return;
  }

  if (millis() - obstaclePhaseStartMs > RETURN_TURN_TIMEOUT_MS)
  {
    fail("Obstacle turn timeout.");
    return;
  }

  int16_t speed = returnTurnSpeedForError(fabs(error));
  pivotDirection(obstacleTurnDirection, speed);
}

void obstacleStraightStep()
{
  if (frontObstacleDetected())
  {
    stopAllMotors();
    obstacleTurnDirection = -obstacleTurnDirection;
    Serial.println("OBS | Detour path blocked. Backing away and trying the opposite side.");
    enterObstaclePhase(OBS_BACK_AWAY);
    return;
  }

  obstacleCountRFID();
  obstacleCountEncoderFallback();

  if (obstacleNodesTravelled >= obstacleNodeTarget)
  {
    stopAllMotors();
    switch (obstaclePhase)
    {
      case OBS_SIDE_STEP_OUT:
        beginObstacleTurn(OBS_TURN_FORWARD, -obstacleTurnDirection);
        break;
      case OBS_PASS_OBSTACLE:
        beginObstacleTurn(OBS_TURN_BACK, -obstacleTurnDirection);
        break;
      case OBS_SIDE_STEP_BACK:
        beginObstacleTurn(OBS_TURN_ORIGINAL, obstacleTurnDirection);
        break;
      default:
        finishObstacleAvoidance();
        break;
    }
    return;
  }

  if (millis() - obstaclePhaseStartMs > OBSTACLE_STEP_TIMEOUT_MS)
  {
    Serial.println("OBS | straight timeout, advancing phase by encoder/time fallback.");
    obstacleNodesTravelled = obstacleNodeTarget;
    return;
  }

  driveYawHold(RETURN_SPEED);
}

void obstacleAvoidStep()
{
  switch (obstaclePhase)
  {
    case OBS_BACK_AWAY:
      driveForward(-OBSTACLE_BACK_AWAY_SPEED);
      if (millis() - obstaclePhaseStartMs >= OBSTACLE_BACK_AWAY_MS)
      {
        stopAllMotors();
        beginObstacleTurn(OBS_TURN_OUT, obstacleTurnDirection);
      }
      break;
    case OBS_TURN_OUT:
    case OBS_TURN_FORWARD:
    case OBS_TURN_BACK:
    case OBS_TURN_ORIGINAL:
      obstacleTurnStep();
      break;
    case OBS_SIDE_STEP_OUT:
    case OBS_PASS_OBSTACLE:
    case OBS_SIDE_STEP_BACK:
      obstacleStraightStep();
      break;
    case OBS_IDLE:
    default:
      finishObstacleAvoidance();
      break;
  }
}

uint8_t activeSensorCount()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (sensorOnLine[i])
    {
      count++;
    }
  }
  return count;
}

void tunnelToBaseStep()
{
  readSensors();
  if (computeLineError(linePidLastError) && millis() - stateStartMs > 900)
  {
    stopAllMotors();
    resetLinePid();
    Serial.println("TUNNEL | Line seen near base. Switching to right-hand-priority line follow.");
    enterState(BASE_RIGHT_PRIORITY);
    return;
  }

  if (millis() - stateStartMs >= TUNNEL_TO_BASE_MS)
  {
    stopAllMotors();
    resetLinePid();
    Serial.println("TUNNEL | Timed exit. Switching to right-hand-priority line follow.");
    enterState(BASE_RIGHT_PRIORITY);
    return;
  }

  float leftCm = leftDistanceCm();
  float rightCm = rightDistanceCm();
  float frontCm = frontDistanceCm();
  float error = 0.0;

  bool leftValid = validWallDistance(leftCm);
  bool rightValid = validWallDistance(rightCm);
  if (frontCm > 0.0 && frontCm < FRONT_OBSTACLE_CM)
  {
    stopAllMotors();
    Serial.print("TUNNEL | Front blocked ");
    Serial.print(frontCm, 1);
    Serial.println("cm, pausing.");
    return;
  }

  if (leftValid && rightValid)
  {
    error = (rightCm - leftCm) * 0.5;
  }
  else if (leftValid)
  {
    error = WALL_TARGET_CM - leftCm;
  }
  else if (rightValid)
  {
    error = rightCm - WALL_TARGET_CM;
  }

  unsigned long now = millis();
  float dtSec = wallHasLast && now > wallLastMs ? (now - wallLastMs) / 1000.0 : 0.02;
  if (dtSec < 0.01)
  {
    dtSec = 0.01;
  }
  float derivative = wallHasLast ? (error - wallLastError) / dtSec : 0.0;
  int16_t correction = constrain((int16_t)(WALL_KP * error + WALL_KD * derivative),
                                 (int16_t)-WALL_MAX_CORRECTION,
                                 (int16_t) WALL_MAX_CORRECTION);
  int16_t base = (leftCm > 0.0 && leftCm < WALL_CRITICAL_CM) ||
                         (rightCm > 0.0 && rightCm < WALL_CRITICAL_CM)
                     ? TUNNEL_MIN_SPEED
                     : TUNNEL_SPEED;

  driveBySide(base + correction, base - correction);

  wallLastError = error;
  wallLastMs = now;
  wallHasLast = true;

  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS)
  {
    lastStatusMs = millis();
    Serial.print("TUNNEL | L=");
    Serial.print(leftCm, 1);
    Serial.print(" R=");
    Serial.print(rightCm, 1);
    Serial.print(" F=");
    Serial.print(frontCm, 1);
    Serial.print(" err=");
    Serial.print(error, 1);
    Serial.print(" corr=");
    Serial.println(correction);
  }
}

void baseRightPriorityStep()
{
  if (returnTurnActive)
  {
    alignStep(BASE_RIGHT_PRIORITY);
    return;
  }

  readSensors();
  if (activeSensorCount() >= RIGHT_PRIORITY_T_ACTIVE_MIN)
  {
    setTargetHeading(headingAfterTurn(robotGridHeading, 1));
    targetNodes = 0;
    Serial.println("BASE | Junction detected. Right-hand priority turn.");
    beginReturnTurn();
    return;
  }

  float lineError = 0.0;
  if (computeLineError(lineError))
  {
    driveLinePid(lineError);
    return;
  }

  driveYawHold(BASE_RIGHT_PRIORITY_SPEED);
}

void printStatus()
{
  if (millis() - lastStatusMs < STATUS_INTERVAL_MS)
  {
    return;
  }

  lastStatusMs = millis();
  Serial.print("STATUS | State=");
  Serial.print(stateName(state));
  Serial.print(" Pos=(");
  Serial.print(robotGridX);
  Serial.print(",");
  Serial.print(robotGridY);
  Serial.print(") Heading=");
  Serial.print(headingName(robotGridHeading));
  Serial.print(" Target=");
  Serial.print(headingName(targetHeading));
  Serial.print(" Node=");
  Serial.print(nodesTravelled);
  Serial.print("/");
  Serial.print(targetNodes);
  Serial.print(" Yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" Guide=");
  Serial.print(blackLineAvailableHere() ? (lastDriveUsedLine ? "LINE" : "LINE_ZONE") : "IMU");
  Serial.print(" RFID=");
  Serial.print(rfidReady ? "OK" : "NA");
  Serial.print(" EncD=");
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    long delta = encoderCounts[i] - lastStatusEncoderCounts[i];
    Serial.print(delta);
    if (i + 1 < ENCODER_COUNT)
    {
      Serial.print(",");
    }
    lastStatusEncoderCounts[i] = encoderCounts[i];
  }
  Serial.print(" Kill=");
  Serial.println(killStopped ? "STOP" : "RUN");
}

void updateButtons()
{
  if (ignoreD45UntilReleased)
  {
    if (digitalRead(KILL_BUTTON_PIN) == HIGH)
    {
      ignoreD45UntilReleased = false;
    }
  }
  else if (digitalRead(KILL_BUTTON_PIN) == LOW)
  {
    if (!killStopped)
    {
      Serial.println("KILL | D45 stop.");
    }
    killStopped = true;
  }

  if (digitalRead(KILL_BUTTON_PIN) == LOW)
  {
    return;
  }

  if (digitalRead(REVIVE_BUTTON_PIN) == LOW)
  {
    if (killStopped)
    {
      Serial.println("REVIVE | D32 run.");
      lastUpdateMicros = micros();
    }
    killStopped = false;
  }

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
}

bool handleStartupWait()
{
  if (!startupWaiting)
  {
    return false;
  }

  stopAllMotors();
  if (millis() - startupLastBlinkMs >= STARTUP_BLINK_MS)
  {
    startupLastBlinkMs = millis();
    startupRedOn = !startupRedOn;
  }
  digitalWrite(LED_RED_PIN, startupRedOn ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  if (digitalRead(KILL_BUTTON_PIN) == LOW)
  {
    startupWaiting = false;
    ignoreD45UntilReleased = true;
    killStopped = false;
    runStartMs = millis();
    stateStartMs = millis();
    nodeStartMs = millis();
    nodesTravelled = 0;
    randomNodesCounted = 0;
    targetNodes = START_FORWARD_NODES;
    serverCoordinateKnown = false;
    waitingForFirstServerCoordinate = false;
    firstTagFallbackUsed = false;
    positionKnown = false;
    haveLastServerCoordinate = false;
    robotGridX = ARENA_ENTRY_X;
    robotGridY = ARENA_ENTRY_Y;
    robotGridHeading = GRID_HEADING_NORTH;
    obstacleDepth = 0;
    haveLastNodeUid = false;
    copyEncoderCounts(nodeStartEncoderCounts);
    copyEncoderCounts(lastStatusEncoderCounts);
    lastUpdateMicros = micros();
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    Serial.println("STARTUP | D45 pressed. Back-to-base test enabled.");
    printPosition("START");
  }

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(REVIVE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(KILL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(US_FRONT_TRIG_PIN, OUTPUT);
  pinMode(US_FRONT_ECHO_PIN, INPUT);
  pinMode(US_LEFT_TRIG_PIN, OUTPUT);
  pinMode(US_LEFT_ECHO_PIN, INPUT);
  pinMode(US_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(US_RIGHT_ECHO_PIN, INPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);

  setupMotoron(mcLower, RF, RR, LR);
  setupMotoron(mcUpper, LF);
  stopAllMotors();
  beginEncoders();

  while (!initIMU())
  {
    Serial.println("IMU | Not ready, retrying...");
    delay(500);
  }
  calibrateGyroZ();
  rfidReady = initRFIDNoResetPin();
  setupServerMessenger();

  robotGridX = ARENA_ENTRY_X;
  robotGridY = ARENA_ENTRY_Y;
  positionKnown = false;
  serverCoordinateKnown = false;
  waitingForFirstServerCoordinate = false;
  haveLastServerCoordinate = false;
  robotGridHeading = GRID_HEADING_NORTH;
  setTargetHeading(GRID_HEADING_NORTH);
  state = RANDOM_FORWARD;
  targetNodes = START_FORWARD_NODES;
  nodesTravelled = 0;
  randomNodesCounted = 0;
  stateStartMs = millis();
  nodeStartMs = millis();
  startupWaiting = true;
  killStopped = true;
  startupRedOn = true;
  startupLastBlinkMs = millis();

  Serial.println("BACK_TO_BASE | Ready.");
  Serial.println("BACK_TO_BASE | Press D45: read random arena RFID, ask server for x/y, return to exit (9,7), then tunnel/base right-priority.");
  Serial.println("BACK_TO_BASE | Entry reference is (9,3), but test route waits for the first server coordinate before planning.");
  Serial.println("BACK_TO_BASE | y<=4 uses black-line PID; y>4 uses IMU yaw hold. RFID/server coordinates correct position.");
  Serial.println("BACK_TO_BASE | Front/side ultrasonic wall scan protects route choices. Obstacle recursion max=3 between RFID anchors.");
  Serial.print("BACK_TO_BASE | Speeds random=");
  Serial.print(RANDOM_FORWARD_SPEED);
  Serial.print(" return=");
  Serial.print(RETURN_SPEED);
  Serial.print(" nodeTicks=");
  Serial.println(NODE_ENCODER_TICKS);
}

void loop()
{
  updateYaw();
  pollEncoders();
  updateRFIDReady();
  updateServerMessenger();

  if (handleStartupWait())
  {
    monitorReflectanceSensors();
    printStatus();
    return;
  }

  updateButtons();
  monitorReflectanceSensors();

  if (killStopped)
  {
    stopAllMotors();
    printStatus();
    return;
  }

  if (state != DONE && state != FAILED && millis() - runStartMs >= RUN_TIME_LIMIT_MS)
  {
    fail("60s timeout.");
  }

  switch (state)
  {
    case RANDOM_FORWARD:
      randomForwardStep();
      break;
    case ALIGN_X:
      alignStep(DRIVE_X);
      break;
    case DRIVE_X:
      driveNodesStep();
      break;
    case ALIGN_Y:
      alignStep(DRIVE_Y);
      break;
    case DRIVE_Y:
      driveNodesStep();
      break;
    case OBSTACLE_AVOID:
      obstacleAvoidStep();
      break;
    case TUNNEL_TO_BASE:
      tunnelToBaseStep();
      break;
    case BASE_RIGHT_PRIORITY:
      baseRightPriorityStep();
      break;
    case DONE:
    case FAILED:
    default:
      stopAllMotors();
      break;
  }

  printStatus();
}
