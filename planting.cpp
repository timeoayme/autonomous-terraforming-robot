// Field seed-planting controller before emergency-return mode.
//
// Build/upload with:
//   pio run -e pre_emergency_planting -t upload
//
// Strategy:
// - First attempt: stable exploration and planting only. No revival.
// - Second attempt: optionally revive stranded robots in open middle areas.
// - Prefer open/central arena movement and avoid edge cells.
// - Treat every RFID tag as a localization and fertility checkpoint.
// - If line tracking is lost, recover toward open space before sweeping.
// - If stuck, self-rescue has priority over every other behavior.

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Servo.h>
#include "ICM_20948.h"
#include "secrets.h"

// Competition / test mode switches.
const bool SECOND_ATTEMPT_MODE = false;
const bool OFFLINE_TEST_MODE = false;
const bool ENABLE_STARTUP_SERVO_TEST = false;
const bool ENABLE_REVIVAL = SECOND_ATTEMPT_MODE;
const bool ALLOW_DESPERATE_INFERTILE_PLANTING = false;

// Server mode. Offline test fakes fertility replies but keeps all motion logic.
const bool USE_SERVER_MESSENGER = !OFFLINE_TEST_MODE;

// Arena assumptions. Server coordinates are expected to be x/y in a 9x9 grid.
const int8_t ARENA_MIN_COORD = 1;
const int8_t ARENA_MAX_COORD = 9;
const int8_t OPEN_TARGET_X = 5;
const int8_t OPEN_TARGET_Y = 7;
const int8_t RETURN_EXIT_X = 9;
const int8_t RETURN_EXIT_Y = 7;
const int8_t EDGE_AVOID_LOW = 2;
const int8_t EDGE_AVOID_HIGH = 9;
const int8_t START_HEADING_NORTH = 0; // +Y after entering the arena.

const unsigned long RUN_LIMIT_MS = 300000UL;          // 5 minutes.
const unsigned long FORCE_RETURN_ELAPSED_MS = 225000UL; // 3:45 elapsed.
const int FORCE_RETURN_TIME_LEFT_SEC = 75;
const uint8_t MAX_SEEDS = 5;

// Motoron stack. Same wiring as main.cpp.
const uint8_t LOWER_MOTORON_ADDR = 0x11;
const uint8_t UPPER_MOTORON_ADDR = 0x10;
MotoronI2C mcLower(LOWER_MOTORON_ADDR);
MotoronI2C mcUpper(UPPER_MOTORON_ADDR);

const uint8_t RF = 1;
const uint8_t RR = 2;
const uint8_t LR = 3;
const uint8_t LF = 1;

// Raw Motoron polarity for physical forward motion.
const int8_t FWD_RF = 1;
const int8_t FWD_RR = -1;
const int8_t FWD_LR = 1;
const int8_t FWD_LF = 1;

// Encoders: RF, RR, LR, LF.
const uint8_t ENCODER_COUNT = 4;
const uint8_t ENC_A[ENCODER_COUNT] = {18, 2, 4, 8};
const uint8_t ENC_B[ENCODER_COUNT] = {19, 3, 5, 9};
volatile long encoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
bool lastEncoderA[ENCODER_COUNT] = {HIGH, HIGH, HIGH, HIGH};

// Reflectance board using OUT1/OUT3/OUT5/OUT7/OUT9, right to left.
const uint8_t SENSOR_COUNT = 5;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {22, 24, 26, 28, 31};
const char *SENSOR_NAMES[SENSOR_COUNT] = {"O1", "O3", "O5", "O7", "O9"};
const uint8_t SENSOR_RIGHT = 0;
const uint8_t SENSOR_SLIGHT_RIGHT = 1;
const uint8_t SENSOR_MIDDLE = 2;
const uint8_t SENSOR_SLIGHT_LEFT = 3;
const uint8_t SENSOR_LEFT = 4;
unsigned int sensorValues[SENSOR_COUNT] = {0, 0, 0, 0, 0};
bool sensorOnLine[SENSOR_COUNT] = {false, false, false, false, false};

// RFID on SDA1/SCL1.
const uint8_t RFID_I2C_ADDR = 0x28;
const uint8_t RFID_UNUSED_RESET_PIN = 0xFF;
const size_t RFID_TAG_ID_MAX_LEN = 24;
MFRC522_I2C mfrc522(RFID_I2C_ADDR, RFID_UNUSED_RESET_PIN, &Wire1);

// WiFi / server.
MiniMessenger serverMessenger;
const char *SERVER_TARGET_BOARD = "server";
const unsigned long SERVER_REGISTER_INTERVAL_MS = 5000;
const unsigned long SERVER_REPLY_TIMEOUT_MS = 2200;
const unsigned long SERVER_STATUS_INTERVAL_MS = 1500;
bool serverMessengerStarted = false;
bool serverLastConnected = false;
bool serverHeartbeatAllowsMovement = true;
bool serverEmergencyActive = false;
bool serverDisableActive = false;
int serverTimeLeftSec = -1;
unsigned long serverLastRegisterMs = 0;
unsigned long serverLastStatusMs = 0;

bool serverRfidWaiting = false;
bool serverRfidReplyReceived = false;
bool serverFertile = false;
bool serverPlanted = false;
bool serverHaveCoord = false;
int8_t serverReplyX = -1;
int8_t serverReplyY = -1;
unsigned long serverRfidRequestMs = 0;
char activeRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char lastRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char plantingRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char pendingSeedReportTagId[RFID_TAG_ID_MAX_LEN] = "";
bool pendingSeedReport = false;
bool rfidReady = false;

// IMU.
ICM_20948_I2C imu;
bool imuAd0Val = true;
float gyroZBias = 0.0;
float yawDeg = 0.0;
float turnStartYawDeg = 0.0;
unsigned long lastUpdateMicros = 0;

// Ultrasonic.
const uint8_t US_FRONT_ECHO = 37;
const uint8_t US_FRONT_TRIG = 39;
const uint8_t US_LEFT_ECHO = 52;
const uint8_t US_LEFT_TRIG = 44;
const uint8_t US_RIGHT_ECHO = 46;
const uint8_t US_RIGHT_TRIG = 48;
const uint32_t ULTRASONIC_TIMEOUT_US = 25000;
float usFrontCm = -1.0;
float usLeftCm = -1.0;
float usRightCm = -1.0;
unsigned long lastUltrasonicMs = 0;
uint8_t ultrasonicSlot = 0;

// Buttons / LEDs / planter.
const uint8_t REVIVE_BUTTON_PIN = 32;
const uint8_t KILL_BUTTON_PIN = 45;
const uint8_t LED_RED_PIN = 36;
const uint8_t LED_GREEN_PIN = 38;
const uint8_t SERVO_SIGNAL_PIN = 33;
const unsigned long DEBOUNCE_MS = 40;
const unsigned long STARTUP_BLINK_MS = 300;

struct DebouncedButton
{
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

DebouncedButton reviveButton = {REVIVE_BUTTON_PIN, HIGH, HIGH, 0};
DebouncedButton killButton = {KILL_BUTTON_PIN, HIGH, HIGH, 0};
bool startupWaiting = true;
bool killStopped = true;
bool ignoreD45UntilReleased = false;
bool redBlinkOn = true;
bool reviveLedGreen = false;
unsigned long lastBlinkMs = 0;

Servo planterServo;
const int PLANTER_MIN_ANGLE = 0;
const int PLANTER_MAX_ANGLE = 135;
const unsigned long PLANTER_STEP_DELAY_MS = 15;
const unsigned long PLANTER_END_PAUSE_MS = 500;
const unsigned long RFID_PLANT_COOLDOWN_MS = 3000;
const unsigned long RFID_PAUSE_MS = 900;
int planterAngle = PLANTER_MIN_ANGLE;
unsigned long planterLastStepMs = 0;
unsigned long planterPauseStartMs = 0;
unsigned long lastPlantTriggerMs = 0;
bool seedReportArmed = false;

enum PlanterState
{
  PLANTER_IDLE,
  PLANTER_SWEEP_FORWARD,
  PLANTER_FORWARD_PAUSE,
  PLANTER_SWEEP_BACKWARD,
  PLANTER_BACKWARD_PAUSE
};

PlanterState planterState = PLANTER_IDLE;

// Motion tuning. Conservative first-attempt defaults.
const int16_t MAX_SPEED = 1100;
const int16_t LINE_SPEED = 220;
const int16_t LINE_EDGE_SPEED = 190;
const int16_t LOST_BACKUP_SPEED = 220;
const int16_t LOST_SPIN_SPEED = 220;
const int16_t OPEN_PUSH_SPEED = 380;
const int16_t SELF_RESCUE_PUSH_SPEED = 600;
const int16_t TURN_FAST_SPEED = 460;
const int16_t TURN_MID_SPEED = 390;
const int16_t TURN_FINE_SPEED = 280;
const int16_t REVIVAL_APPROACH_SPEED = 400;
const int16_t REVIVAL_BACKUP_SPEED = 320;

const unsigned int SENSOR_TIMEOUT_US = 3000;
// The lowered reflectance board sees real black-line values below the old
// 280us threshold. Acquisition is permissive; branch detection stays strict.
const unsigned int LINE_THRESHOLD = 220;
const unsigned int LINE_SIGNAL_FLOOR = 180;
const int LINE_MIN_SIGNAL_TOTAL = 20;
const float LINE_PID_KP = 0.22;
const float LINE_PID_KI = 0.025;
const float LINE_PID_KD = 0.006;
const float LINE_PID_INTEGRAL_LIMIT = 2500.0;
const int16_t LINE_PID_MAX_CORRECTION = 300;
const bool BLACK_LINE_IS_HIGH = true;
const unsigned int BRANCH_SIDE_THRESHOLD = 300;
const int BRANCH_SCORE_MIN = 560;
const unsigned long BRANCH_COOLDOWN_MS = 900;
const float TURN_TARGET_DEG = 90.0;
const float TURN_TOLERANCE_DEG = 3.0;
const unsigned long TURN_TIMEOUT_MS = 9000;
const unsigned long TURN_SETTLE_MS = 160;
const unsigned long POST_TURN_IGNORE_MS = 500;
const unsigned long RFID_CHECK_INTERVAL_MS = 80;
const unsigned long RFID_CARD_COOLDOWN_MS = 1200;
const unsigned long RFID_RETRY_INTERVAL_MS = 1000;
const unsigned long RFID_ABSENT_CONFIRM_MS = 500;
const unsigned long RFID_DUPLICATE_PRINT_MS = 1000;
const unsigned long RFID_NODE_REVISIT_COOLDOWN_MS = 30000;
const uint8_t RFID_RECENT_NODE_COUNT = 6;
const unsigned long STATUS_INTERVAL_MS = 500;
const unsigned long LINE_LOST_CONFIRM_MS = 1200;
const unsigned long NO_LINE_SELF_RESCUE_AFTER_MS = 10000;
const unsigned long LINE_RECOVERY_STABLE_MS = 300;
const int16_t LINE_GAP_CROSS_SPEED = 180;

// The RFID antenna reaches the node before the seed outlet. Move the outlet
// about 1.3 cm beyond the detected tag before operating the planter.
const int16_t PLANT_APPROACH_SPEED = 240;
const unsigned long PLANT_APPROACH_MS = 210;

const float FRONT_OBSTACLE_CM = 13.0;
const float REVIVAL_CONTACT_CM = 5.0;
const unsigned long REVIVAL_APPROACH_TIMEOUT_MS = 1100;
const unsigned long REVIVAL_WAIT_MS = 7000;
const long NODE_BACKUP_ENCODER_TICKS = 150;

const unsigned long LOST_BACKUP_MS = 950;
const unsigned long LOST_SPIN_MS = 2300;
const unsigned long OPEN_PUSH_MS = 700;
const float LOST_SPIN_TARGET_DEG = 90.0;
const int16_t FULL_SEARCH_SPIN_SPEED = 220;
const float FULL_SEARCH_TARGET_DEG = 355.0;
const float FULL_SEARCH_SEGMENT_DEG = 15.0;
const unsigned long FULL_SEARCH_SEGMENT_PAUSE_MS = 110;
const unsigned long FULL_SEARCH_TIMEOUT_MS = 30000;
const unsigned long FULL_SEARCH_PRINT_MS = 400;
const float IMU_TURN_PROGRESS_MIN_DEG = 1.5;
const unsigned long IMU_TURN_STALL_MS = 1500;

const unsigned long STUCK_SAMPLE_MS = 500;
const unsigned long STUCK_CONFIRM_MS = 1700;
const long STUCK_MIN_ENCODER_TICKS = 2;
const float STUCK_MIN_YAW_DELTA_DEG = 0.6;
const int16_t RETURN_OPEN_SPEED = 250;
const int16_t RETURN_TUNNEL_SPEED = 360;
const int16_t RETURN_TUNNEL_MIN_SPEED = 260;
const int16_t RETURN_TUNNEL_MAX_CORRECTION = 130;
const float RETURN_YAW_KP = 8.0;
const float RETURN_TUNNEL_KP = 12.0;
const float RETURN_TUNNEL_CRITICAL_CM = 4.5;
const unsigned long RETURN_TUNNEL_MS = 5000;

enum FieldState
{
  FIELD_EXPLORE_LINE,
  FIELD_TURNING,
  FIELD_RFID_WAIT,
  FIELD_PRE_PLANT_FORWARD,
  FIELD_PLANTING_WAIT,
  FIELD_POST_NODE_DECIDE,
  FIELD_NO_LINE_BACKUP,
  FIELD_NO_LINE_SPIN,
  FIELD_NO_LINE_FULL_SPIN,
  FIELD_NO_LINE_OPEN_PUSH,
  FIELD_REVIVAL_APPROACH,
  FIELD_REVIVAL_WAIT,
  FIELD_REVIVAL_BACKUP_NODE,
  FIELD_SELF_RESCUE_BACKUP,
  FIELD_SELF_RESCUE_SPIN,
  FIELD_SELF_RESCUE_OPEN_PUSH,
  FIELD_RETURN_REQUESTED,
  FIELD_RETURN_TUNNEL,
  FIELD_DONE
};

FieldState state = FIELD_EXPLORE_LINE;
unsigned long stateStartMs = 0;
unsigned long runStartMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastRFIDCheckMs = 0;
unsigned long lastRFIDActionMs = 0;
unsigned long lastRFIDRetryMs = 0;
unsigned long rfidAbsentSinceMs = 0;
unsigned long lastRFIDDuplicatePrintMs = 0;
bool rfidLeftLastAcceptedTag = true;
char lastAcceptedRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char recentRFIDTagIds[RFID_RECENT_NODE_COUNT][RFID_TAG_ID_MAX_LEN] = {};
unsigned long recentRFIDSeenMs[RFID_RECENT_NODE_COUNT] = {};
uint8_t nextRecentRFIDSlot = 0;
unsigned long lineLostSinceMs = 0;
unsigned long noLineEpisodeStartMs = 0;
unsigned long lineRecoveryCandidateSinceMs = 0;
bool lineGapPrinted = false;
unsigned long branchCooldownUntilMs = 0;
unsigned long postTurnIgnoreUntilMs = 0;
unsigned long turnSettleStartMs = 0;
unsigned long lastTurnPrintMs = 0;
float turnLastProgressDeg = 0.0;
unsigned long turnLastProgressMs = 0;
uint8_t seedsPlanted = 0;
uint8_t infertileSeen = 0;
uint8_t fertileSeen = 0;
uint8_t nodesScanned = 0;

int8_t currentX = -1;
int8_t currentY = -1;
bool positionKnown = false;
int8_t heading = START_HEADING_NORTH; // 0=N, 1=E, 2=S, 3=W.
bool returnModeActive = false;
uint8_t returnQuarterTurnsPending = 0;
int returnPendingTurnDirection = 1;

int turnDirection = 0; // -1 left, +1 right.
float turnTargetDeg = TURN_TARGET_DEG;
bool turnSettling = false;

float linePidIntegral = 0.0;
float linePidLastError = 0.0;
unsigned long linePidLastMs = 0;
bool linePidHasLast = false;
int lastLineDirection = 0;

long rescueStartEncoder[ENCODER_COUNT] = {0, 0, 0, 0};
int openBiasDirection = 0; // -1 left, +1 right.
uint8_t noLineSpinAttempts = 0;
int noLineSearchDirection = 0;
float noLineSpinStartYawDeg = 0.0;
float noLineSpinLastProgressDeg = 0.0;
unsigned long noLineSpinLastProgressMs = 0;
float fullSearchLastYawDeg = 0.0;
float fullSearchAccumulatedDeg = 0.0;
float fullSearchSegmentAccumulatedDeg = 0.0;
bool fullSearchSegmentPaused = false;
unsigned long fullSearchSegmentPauseStartMs = 0;
unsigned long lastFullSearchPrintMs = 0;
unsigned long fullSearchLastProgressMs = 0;

int16_t lastMotorCommand[ENCODER_COUNT] = {0, 0, 0, 0};
long stuckSampleEncoder[ENCODER_COUNT] = {0, 0, 0, 0};
float stuckSampleYaw = 0.0;
unsigned long stuckSampleMs = 0;
unsigned long stuckSinceMs = 0;

void printSensorsAndStatus();
void planReturnStep();

const char *stateName(FieldState s)
{
  switch (s)
  {
    case FIELD_EXPLORE_LINE: return "EXPLORE_LINE";
    case FIELD_TURNING: return "TURNING";
    case FIELD_RFID_WAIT: return "RFID_WAIT";
    case FIELD_PRE_PLANT_FORWARD: return "PRE_PLANT_FORWARD";
    case FIELD_PLANTING_WAIT: return "PLANTING_WAIT";
    case FIELD_POST_NODE_DECIDE: return "POST_NODE_DECIDE";
    case FIELD_NO_LINE_BACKUP: return "NO_LINE_BACKUP";
    case FIELD_NO_LINE_SPIN: return "NO_LINE_SPIN";
    case FIELD_NO_LINE_FULL_SPIN: return "NO_LINE_FULL_SPIN";
    case FIELD_NO_LINE_OPEN_PUSH: return "NO_LINE_OPEN_PUSH";
    case FIELD_REVIVAL_APPROACH: return "REVIVAL_APPROACH";
    case FIELD_REVIVAL_WAIT: return "REVIVAL_WAIT";
    case FIELD_REVIVAL_BACKUP_NODE: return "REVIVAL_BACKUP_NODE";
    case FIELD_SELF_RESCUE_BACKUP: return "SELF_RESCUE_BACKUP";
    case FIELD_SELF_RESCUE_SPIN: return "SELF_RESCUE_SPIN";
    case FIELD_SELF_RESCUE_OPEN_PUSH: return "SELF_RESCUE_OPEN_PUSH";
    case FIELD_RETURN_REQUESTED: return "RETURN_REQUESTED";
    case FIELD_RETURN_TUNNEL: return "RETURN_TUNNEL";
    case FIELD_DONE: return "DONE";
  }
  return "UNKNOWN";
}

void enterState(FieldState next)
{
  if (next != state)
  {
    Serial.print("STATE | ");
    Serial.println(stateName(next));
  }
  if (next != FIELD_EXPLORE_LINE)
  {
    lineLostSinceMs = 0;
    lineGapPrinted = false;
  }
  if ((next == FIELD_NO_LINE_BACKUP ||
       next == FIELD_NO_LINE_SPIN ||
       next == FIELD_NO_LINE_FULL_SPIN ||
       next == FIELD_NO_LINE_OPEN_PUSH) &&
      noLineEpisodeStartMs == 0)
  {
    noLineEpisodeStartMs = millis();
    lineRecoveryCandidateSinceMs = 0;
    Serial.println("NO_LINE_TIMER | Started. Self rescue after 10000ms without stable line.");
  }
  if (next == FIELD_NO_LINE_SPIN)
  {
    noLineSpinStartYawDeg = yawDeg;
    noLineSpinLastProgressDeg = 0.0;
    noLineSpinLastProgressMs = millis();
    noLineSearchDirection = noLineSpinAttempts == 0 ? openBiasDirection : -openBiasDirection;
    if (noLineSearchDirection == 0)
    {
      noLineSearchDirection = 1;
    }
  }
  else if (next == FIELD_NO_LINE_FULL_SPIN)
  {
    noLineSearchDirection = openBiasDirection == 0 ? 1 : openBiasDirection;
    fullSearchLastYawDeg = yawDeg;
    fullSearchAccumulatedDeg = 0.0;
    fullSearchSegmentAccumulatedDeg = 0.0;
    fullSearchSegmentPaused = false;
    fullSearchSegmentPauseStartMs = 0;
    lastFullSearchPrintMs = 0;
    fullSearchLastProgressMs = millis();
  }
  state = next;
  stateStartMs = millis();
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

bool parseBoolField(const char *message, const char *key, bool &value)
{
  char text[12];
  if (!extractField(message, key, text, sizeof(text)))
  {
    return false;
  }
  if (strcmp(text, "true") == 0 || strcmp(text, "1") == 0 || strcmp(text, "yes") == 0)
  {
    value = true;
    return true;
  }
  if (strcmp(text, "false") == 0 || strcmp(text, "0") == 0 || strcmp(text, "no") == 0)
  {
    value = false;
    return true;
  }
  return false;
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
  }
}

void setupMotoron(MotoronI2C &mc, uint8_t m1, uint8_t m2 = 0, uint8_t m3 = 0)
{
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(m1, 800);
  mc.setMaxDeceleration(m1, 900);
  if (m2 != 0)
  {
    mc.setMaxAcceleration(m2, 800);
    mc.setMaxDeceleration(m2, 900);
  }
  if (m3 != 0)
  {
    mc.setMaxAcceleration(m3, 800);
    mc.setMaxDeceleration(m3, 900);
  }
}

void setMotorSpeeds(int16_t rf, int16_t rr, int16_t lr, int16_t lf)
{
  lastMotorCommand[0] = constrain(rf, -MAX_SPEED, MAX_SPEED);
  lastMotorCommand[1] = constrain(rr, -MAX_SPEED, MAX_SPEED);
  lastMotorCommand[2] = constrain(lr, -MAX_SPEED, MAX_SPEED);
  lastMotorCommand[3] = constrain(lf, -MAX_SPEED, MAX_SPEED);
  mcLower.setSpeed(RF, lastMotorCommand[0]);
  mcLower.setSpeed(RR, lastMotorCommand[1]);
  mcLower.setSpeed(LR, lastMotorCommand[2]);
  mcUpper.setSpeed(LF, lastMotorCommand[3]);
}

void stopAllMotors()
{
  setMotorSpeeds(0, 0, 0, 0);
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

void pivotRight(int16_t speed)
{
  setMotorSpeeds(
      -speed * FWD_RF,
      -speed * FWD_RR,
       speed * FWD_LR,
       speed * FWD_LF);
}

void pivotLeft(int16_t speed)
{
  setMotorSpeeds(
       speed * FWD_RF,
       speed * FWD_RR,
      -speed * FWD_LR,
      -speed * FWD_LF);
}

void beginButton(DebouncedButton &button)
{
  pinMode(button.pin, INPUT_PULLUP);
  button.stableState = digitalRead(button.pin);
  button.lastReading = button.stableState;
  button.lastChangeMs = millis();
}

bool buttonPressed(DebouncedButton &button)
{
  bool reading = digitalRead(button.pin);
  if (reading != button.lastReading)
  {
    button.lastChangeMs = millis();
    button.lastReading = reading;
  }

  if (millis() - button.lastChangeMs > DEBOUNCE_MS && reading != button.stableState)
  {
    button.stableState = reading;
    return button.stableState == LOW;
  }
  return false;
}

void updateLed()
{
  if (startupWaiting || killStopped)
  {
    if (millis() - lastBlinkMs >= STARTUP_BLINK_MS)
    {
      lastBlinkMs = millis();
      redBlinkOn = !redBlinkOn;
    }
    digitalWrite(LED_RED_PIN, redBlinkOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }

  digitalWrite(LED_RED_PIN, reviveLedGreen ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, reviveLedGreen ? HIGH : LOW);
}

void setKillStopped(bool stopped, const char *source)
{
  if (stopped == killStopped)
  {
    if (stopped)
    {
      stopAllMotors();
    }
    return;
  }
  killStopped = stopped;
  Serial.print("KILL | ");
  Serial.print(source);
  Serial.print(" -> ");
  Serial.println(killStopped ? "STOP" : "RUN");
  if (killStopped)
  {
    stopAllMotors();
  }
  else
  {
    lastUpdateMicros = micros();
  }
}

void updateButtons()
{
  if (digitalRead(REVIVE_BUTTON_PIN) == LOW || buttonPressed(reviveButton))
  {
    reviveLedGreen = true;
    Serial.println("REVIVE | D32/front button -> LED GREEN");
  }

  if (ignoreD45UntilReleased)
  {
    if (digitalRead(KILL_BUTTON_PIN) == HIGH)
    {
      ignoreD45UntilReleased = false;
    }
  }
  else if (buttonPressed(killButton))
  {
    if (startupWaiting)
    {
      startupWaiting = false;
      killStopped = false;
      runStartMs = millis();
      enterState(FIELD_EXPLORE_LINE);
      Serial.println("START | D45 pressed. Field planting run begins.");
    }
    else
    {
      setKillStopped(!killStopped, "D45");
    }
  }
}

bool initIMU()
{
  const bool ad0Values[] = {false, true};
  for (uint8_t i = 0; i < 2; i++)
  {
    imuAd0Val = ad0Values[i];
    imu.begin(Wire, imuAd0Val);
    if (imu.status == ICM_20948_Stat_Ok)
    {
      Serial.print("IMU | Ready at 0x");
      Serial.println(imuAd0Val ? "69" : "68");
      return true;
    }
    delay(100);
  }
  return false;
}

void calibrateGyroZ()
{
  Serial.println("IMU | Calibrating gyro Z. Keep still.");
  const int samples = 300;
  float sum = 0.0;
  int count = 0;
  unsigned long startMs = millis();
  while (count < samples && millis() - startMs < 6000)
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

float normalizedAngleDeltaDeg(float currentDeg, float referenceDeg)
{
  float delta = currentDeg - referenceDeg;
  while (delta > 180.0)
  {
    delta -= 360.0;
  }
  while (delta < -180.0)
  {
    delta += 360.0;
  }
  return delta;
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

bool anySensorOnLine()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (sensorOnLine[i])
    {
      return true;
    }
  }
  return false;
}

bool computeLinePidError(float &error)
{
  const int weights[SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000};
  long weightedSum = 0;
  int signalTotal = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    int signal = sensorValues[i] > LINE_SIGNAL_FLOOR ? (int)(sensorValues[i] - LINE_SIGNAL_FLOOR) : 0;
    weightedSum += (long)signal * weights[i];
    signalTotal += signal;
  }
  if (signalTotal < LINE_MIN_SIGNAL_TOTAL)
  {
    return false;
  }
  error = (float)weightedSum / (float)signalTotal;
  return true;
}

void resetLinePid()
{
  linePidIntegral = 0.0;
  linePidLastError = 0.0;
  linePidLastMs = 0;
  linePidHasLast = false;
}

void driveLinePid(float error)
{
  unsigned long now = millis();
  float dtSec = 0.02;
  if (linePidHasLast && now > linePidLastMs)
  {
    dtSec = (float)(now - linePidLastMs) / 1000.0;
    dtSec = constrain(dtSec, 0.005f, 0.1f);
  }
  linePidIntegral += error * dtSec;
  linePidIntegral = constrain(linePidIntegral, -LINE_PID_INTEGRAL_LIMIT, LINE_PID_INTEGRAL_LIMIT);
  float derivative = linePidHasLast ? (error - linePidLastError) / dtSec : 0.0;
  float correctionFloat = LINE_PID_KP * error + LINE_PID_KI * linePidIntegral + LINE_PID_KD * derivative;
  int16_t correction = (int16_t)constrain(correctionFloat, (float)-LINE_PID_MAX_CORRECTION, (float)LINE_PID_MAX_CORRECTION);
  int16_t baseSpeed = positionKnown && (currentX <= EDGE_AVOID_LOW || currentX >= EDGE_AVOID_HIGH ||
                                        currentY <= EDGE_AVOID_LOW || currentY >= EDGE_AVOID_HIGH)
                          ? LINE_EDGE_SPEED
                          : LINE_SPEED;
  int16_t leftSpeed = constrain((int16_t)(baseSpeed - correction), -MAX_SPEED, MAX_SPEED);
  int16_t rightSpeed = constrain((int16_t)(baseSpeed + correction), -MAX_SPEED, MAX_SPEED);
  driveBySide(leftSpeed, rightSpeed);
  linePidLastError = error;
  linePidLastMs = now;
  linePidHasLast = true;
  if (error > 120.0)
  {
    lastLineDirection = -1;
  }
  else if (error < -120.0)
  {
    lastLineDirection = 1;
  }
  else
  {
    lastLineDirection = 0;
  }
}

bool branchSeen(bool &leftBranch, bool &rightBranch)
{
  int rightScore = sensorValues[SENSOR_RIGHT] + sensorValues[SENSOR_SLIGHT_RIGHT];
  int leftScore = sensorValues[SENSOR_SLIGHT_LEFT] + sensorValues[SENSOR_LEFT];
  rightBranch = sensorValues[SENSOR_RIGHT] >= BRANCH_SIDE_THRESHOLD &&
                sensorValues[SENSOR_SLIGHT_RIGHT] >= BRANCH_SIDE_THRESHOLD &&
                rightScore >= BRANCH_SCORE_MIN;
  leftBranch = sensorValues[SENSOR_SLIGHT_LEFT] >= BRANCH_SIDE_THRESHOLD &&
               sensorValues[SENSOR_LEFT] >= BRANCH_SIDE_THRESHOLD &&
               leftScore >= BRANCH_SCORE_MIN;
  return leftBranch || rightBranch;
}

bool coordIsEdgeRisk(int8_t x, int8_t y)
{
  return x <= EDGE_AVOID_LOW || x >= EDGE_AVOID_HIGH ||
         y <= EDGE_AVOID_LOW || y >= EDGE_AVOID_HIGH;
}

void projectStep(int8_t x, int8_t y, int8_t h, int8_t &outX, int8_t &outY)
{
  outX = x;
  outY = y;
  if (h == 0)
  {
    outY++;
  }
  else if (h == 1)
  {
    outX++;
  }
  else if (h == 2)
  {
    outY--;
  }
  else
  {
    outX--;
  }
}

int scoreProjectedMove(int turn)
{
  if (!positionKnown)
  {
    if (turn < 0)
    {
      return 0; // Before localization, softly favor left/front to reach open field.
    }
    if (turn == 0)
    {
      return 2;
    }
    return 4;
  }

  int8_t projectedHeading = (heading + turn + 4) % 4;
  int8_t nx = currentX;
  int8_t ny = currentY;
  projectStep(currentX, currentY, projectedHeading, nx, ny);
  int targetX = returnModeActive ? RETURN_EXIT_X : OPEN_TARGET_X;
  int targetY = returnModeActive ? RETURN_EXIT_Y : OPEN_TARGET_Y;
  int score = abs(nx - targetX) + abs(ny - targetY);
  if (nx < ARENA_MIN_COORD || nx > ARENA_MAX_COORD ||
      ny < ARENA_MIN_COORD || ny > ARENA_MAX_COORD)
  {
    score += 50;
  }
  if (!returnModeActive && coordIsEdgeRisk(nx, ny))
  {
    score += 12;
  }
  return score;
}

int chooseExploreTurn(bool leftAvailable, bool rightAvailable, bool straightAvailable)
{
  int bestTurn = straightAvailable ? 0 : (leftAvailable ? -1 : 1);
  int bestScore = 999;
  const int turns[3] = {-1, 0, 1};
  for (uint8_t i = 0; i < 3; i++)
  {
    int t = turns[i];
    bool available = (t < 0 && leftAvailable) ||
                     (t == 0 && straightAvailable) ||
                     (t > 0 && rightAvailable);
    if (!available)
    {
      continue;
    }
    int score = scoreProjectedMove(t) + random(0, 4);
    if (t == 0 && (leftAvailable || rightAvailable))
    {
      score += 3; // Explore more at confirmed branches without blind turns.
    }
    if (score < bestScore)
    {
      bestScore = score;
      bestTurn = t;
    }
  }
  return bestTurn;
}

void updateHeadingAfterTurn(int direction)
{
  if (direction > 0)
  {
    heading = (heading + 1) % 4;
  }
  else if (direction < 0)
  {
    heading = (heading + 3) % 4;
  }
}

void beginTurn(int direction, const char *reason)
{
  if (direction == 0)
  {
    branchCooldownUntilMs = millis() + BRANCH_COOLDOWN_MS;
    return;
  }
  stopAllMotors();
  resetLinePid();
  turnDirection = direction;
  turnStartYawDeg = yawDeg;
  turnTargetDeg = TURN_TARGET_DEG;
  turnSettling = false;
  turnSettleStartMs = 0;
  lastTurnPrintMs = 0;
  turnLastProgressDeg = 0.0;
  turnLastProgressMs = millis();
  Serial.print("TURN | ");
  Serial.print(direction < 0 ? "LEFT_90" : "RIGHT_90");
  Serial.print(" reason=");
  Serial.println(reason);
  enterState(FIELD_TURNING);
}

void handleTurning()
{
  float delta = fabs(normalizedAngleDeltaDeg(yawDeg, turnStartYawDeg));
  float error = turnTargetDeg - delta;
  if (delta >= turnLastProgressDeg + IMU_TURN_PROGRESS_MIN_DEG)
  {
    turnLastProgressDeg = delta;
    turnLastProgressMs = millis();
  }
  if (!turnSettling && millis() - turnLastProgressMs >= IMU_TURN_STALL_MS)
  {
    stopAllMotors();
    Serial.println("TURN_FAULT | IMU yaw stalled while motors commanded. Enter self rescue.");
    enterState(FIELD_SELF_RESCUE_BACKUP);
    copyEncoderCounts(rescueStartEncoder);
    return;
  }
  if (error <= TURN_TOLERANCE_DEG)
  {
    stopAllMotors();
    if (!turnSettling)
    {
      turnSettling = true;
      turnSettleStartMs = millis();
      updateHeadingAfterTurn(turnDirection);
      Serial.print("TURN | reached delta=");
      Serial.print(delta, 1);
      Serial.print(" heading=");
      Serial.println(heading);
    }
    if (millis() - turnSettleStartMs >= TURN_SETTLE_MS)
    {
      branchCooldownUntilMs = millis() + BRANCH_COOLDOWN_MS;
      postTurnIgnoreUntilMs = millis() + POST_TURN_IGNORE_MS;
      if (returnModeActive && returnQuarterTurnsPending > 0)
      {
        returnQuarterTurnsPending--;
        beginTurn(returnPendingTurnDirection, "return U-turn second quarter");
        return;
      }
      enterState(FIELD_EXPLORE_LINE);
    }
    return;
  }

  if (millis() - stateStartMs >= TURN_TIMEOUT_MS)
  {
    Serial.println("TURN | timeout -> self rescue");
    enterState(FIELD_SELF_RESCUE_BACKUP);
    copyEncoderCounts(rescueStartEncoder);
    return;
  }

  int16_t speed = error > 28.0 ? TURN_FAST_SPEED : (error > 9.0 ? TURN_MID_SPEED : TURN_FINE_SPEED);
  if (turnDirection < 0)
  {
    pivotLeft(speed);
  }
  else
  {
    pivotRight(speed);
  }

  if (millis() - lastTurnPrintMs >= 250)
  {
    lastTurnPrintMs = millis();
    Serial.print("TURN | yaw=");
    Serial.print(yawDeg, 1);
    Serial.print(" delta=");
    Serial.print(delta, 1);
    Serial.print(" err=");
    Serial.print(error, 1);
    Serial.print(" speed=");
    Serial.println(speed);
  }
}

float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0)
  {
    return -1.0;
  }
  return duration * 0.0343 / 2.0;
}

void updateUltrasonic()
{
  if (millis() - lastUltrasonicMs < 45)
  {
    return;
  }
  lastUltrasonicMs = millis();
  if (ultrasonicSlot == 0)
  {
    usFrontCm = readUltrasonicCm(US_FRONT_TRIG, US_FRONT_ECHO);
  }
  else if (ultrasonicSlot == 1)
  {
    usLeftCm = readUltrasonicCm(US_LEFT_TRIG, US_LEFT_ECHO);
  }
  else
  {
    usRightCm = readUltrasonicCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
  }
  ultrasonicSlot = (ultrasonicSlot + 1) % 3;
}

bool validDistance(float cm)
{
  return cm > 0.0 && cm < 400.0;
}

int chooseOpenDirection()
{
  if (validDistance(usLeftCm) && validDistance(usRightCm))
  {
    return usLeftCm >= usRightCm ? -1 : 1;
  }
  if (validDistance(usLeftCm))
  {
    return -1;
  }
  if (validDistance(usRightCm))
  {
    return 1;
  }
  if (positionKnown)
  {
    return currentX > OPEN_TARGET_X ? -1 : 1;
  }
  return lastLineDirection != 0 ? lastLineDirection : -1;
}

bool frontObstacleDetected()
{
  return validDistance(usFrontCm) && usFrontCm <= FRONT_OBSTACLE_CM;
}

bool goodMiddleAreaForRevival()
{
  if (positionKnown)
  {
    return !coordIsEdgeRisk(currentX, currentY);
  }
  return validDistance(usLeftCm) && validDistance(usRightCm) &&
         usLeftCm >= 18.0 && usRightCm >= 18.0;
}

void startOpenSpaceRecovery(const char *reason)
{
  stopAllMotors();
  resetLinePid();
  if (noLineEpisodeStartMs == 0)
  {
    noLineEpisodeStartMs = millis();
    Serial.println("NO_LINE_TIMER | Started. Self rescue after 10000ms without stable line.");
  }
  lineRecoveryCandidateSinceMs = 0;
  openBiasDirection = chooseOpenDirection();
  noLineSpinAttempts = 0;
  Serial.print("NO_LINE | ");
  Serial.print(reason);
  Serial.print(" -> backup toward open ");
  Serial.println(openBiasDirection < 0 ? "LEFT" : "RIGHT");
  enterState(FIELD_NO_LINE_BACKUP);
}

void driveBackwardWithOpenBias(int16_t speed)
{
  int16_t left = -speed;
  int16_t right = -speed;
  if (openBiasDirection < 0)
  {
    left = (int16_t)(-speed * 0.65);
    right = (int16_t)(-speed * 1.15);
  }
  else if (openBiasDirection > 0)
  {
    left = (int16_t)(-speed * 1.15);
    right = (int16_t)(-speed * 0.65);
  }
  driveBySide(left, right);
}

bool lineRecovered()
{
  readSensors();
  if (!anySensorOnLine())
  {
    lineRecoveryCandidateSinceMs = 0;
    return false;
  }

  // Stop on the first glimpse so a thin/noisy line cannot be crossed before
  // it has been confirmed as a usable recovery.
  stopAllMotors();
  if (lineRecoveryCandidateSinceMs == 0)
  {
    lineRecoveryCandidateSinceMs = millis();
    Serial.println("NO_LINE | Candidate line seen; confirming stability.");
    return false;
  }

  if (millis() - lineRecoveryCandidateSinceMs < LINE_RECOVERY_STABLE_MS)
  {
    return false;
  }

  noLineEpisodeStartMs = 0;
  lineRecoveryCandidateSinceMs = 0;
  Serial.println("NO_LINE_TIMER | Stable line confirmed; timer cleared.");
  return true;
}

bool noLineSelfRescueDue()
{
  return noLineEpisodeStartMs != 0 &&
         millis() - noLineEpisodeStartMs >= NO_LINE_SELF_RESCUE_AFTER_MS;
}

void startNoLineSelfRescue(const char *reason)
{
  stopAllMotors();
  openBiasDirection = chooseOpenDirection();
  copyEncoderCounts(rescueStartEncoder);
  Serial.print("NO_LINE_ESCALATE | ");
  Serial.print(reason);
  Serial.print(" open=");
  Serial.println(openBiasDirection < 0 ? "LEFT" : "RIGHT");
  noLineEpisodeStartMs = 0;
  lineRecoveryCandidateSinceMs = 0;
  enterState(FIELD_SELF_RESCUE_BACKUP);
}

void handleNoLineRecovery()
{
  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS)
  {
    readSensors();
    printSensorsAndStatus();
  }

  if (noLineSelfRescueDue())
  {
    startNoLineSelfRescue("No stable line for 10000ms -> self rescue.");
    return;
  }

  if (state == FIELD_NO_LINE_BACKUP)
  {
    driveBackwardWithOpenBias(LOST_BACKUP_SPEED);
    if (millis() - stateStartMs >= LOST_BACKUP_MS)
    {
      stopAllMotors();
      enterState(FIELD_NO_LINE_SPIN);
    }
    return;
  }

  if (state == FIELD_NO_LINE_SPIN)
  {
    if (lineRecovered())
    {
      Serial.println("NO_LINE | Line recovered during 90-degree search.");
      noLineSpinAttempts = 0;
      resetLinePid();
      enterState(FIELD_EXPLORE_LINE);
      return;
    }
    if (noLineSearchDirection < 0)
    {
      pivotLeft(LOST_SPIN_SPEED);
    }
    else
    {
      pivotRight(LOST_SPIN_SPEED);
    }
    float searchedDeg = fabs(normalizedAngleDeltaDeg(yawDeg, noLineSpinStartYawDeg));
    if (searchedDeg >= noLineSpinLastProgressDeg + IMU_TURN_PROGRESS_MIN_DEG)
    {
      noLineSpinLastProgressDeg = searchedDeg;
      noLineSpinLastProgressMs = millis();
    }
    if (millis() - noLineSpinLastProgressMs >= IMU_TURN_STALL_MS)
    {
      startNoLineSelfRescue("IMU yaw stalled during 90-degree search.");
      return;
    }
    if (searchedDeg >= LOST_SPIN_TARGET_DEG || millis() - stateStartMs >= LOST_SPIN_MS)
    {
      stopAllMotors();
      noLineSpinAttempts++;
      Serial.print("NO_LINE | 90-degree search failed attempt=");
      Serial.println(noLineSpinAttempts);
      if (noLineSpinAttempts >= 2)
      {
        Serial.println("NO_LINE | Two searches failed. Start slow segmented 360-degree search.");
        enterState(FIELD_NO_LINE_FULL_SPIN);
      }
      else
      {
        enterState(FIELD_NO_LINE_OPEN_PUSH);
      }
    }
    return;
  }

  if (state == FIELD_NO_LINE_FULL_SPIN)
  {
    if (lineRecovered())
    {
      Serial.print("NO_LINE | Line recovered during 360-degree search at ");
      Serial.print(fullSearchAccumulatedDeg, 1);
      Serial.println("deg.");
      noLineSpinAttempts = 0;
      resetLinePid();
      enterState(FIELD_EXPLORE_LINE);
      return;
    }

    float stepDeg = fabs(normalizedAngleDeltaDeg(yawDeg, fullSearchLastYawDeg));
    fullSearchLastYawDeg = yawDeg;
    if (stepDeg <= 20.0)
    {
      fullSearchAccumulatedDeg += stepDeg;
      fullSearchSegmentAccumulatedDeg += stepDeg;
      if (stepDeg >= 0.15)
      {
        fullSearchLastProgressMs = millis();
      }
    }

    if (!fullSearchSegmentPaused &&
        millis() - fullSearchLastProgressMs >= IMU_TURN_STALL_MS)
    {
      startNoLineSelfRescue("IMU yaw stalled during 360-degree search.");
      return;
    }

    if (fullSearchAccumulatedDeg >= FULL_SEARCH_TARGET_DEG ||
        millis() - stateStartMs >= FULL_SEARCH_TIMEOUT_MS)
    {
      stopAllMotors();
      Serial.println("NO_LINE | Full 360-degree search completed without line. Push toward open space.");
      noLineSpinAttempts = 0;
      enterState(FIELD_NO_LINE_OPEN_PUSH);
      return;
    }

    if (fullSearchSegmentPaused)
    {
      stopAllMotors();
      if (millis() - fullSearchSegmentPauseStartMs >= FULL_SEARCH_SEGMENT_PAUSE_MS)
      {
        fullSearchSegmentPaused = false;
        fullSearchSegmentAccumulatedDeg = 0.0;
      }
      return;
    }

    if (fullSearchSegmentAccumulatedDeg >= FULL_SEARCH_SEGMENT_DEG)
    {
      stopAllMotors();
      fullSearchSegmentPaused = true;
      fullSearchSegmentPauseStartMs = millis();
      return;
    }

    if (noLineSearchDirection < 0)
    {
      pivotLeft(FULL_SEARCH_SPIN_SPEED);
    }
    else
    {
      pivotRight(FULL_SEARCH_SPIN_SPEED);
    }

    if (millis() - lastFullSearchPrintMs >= FULL_SEARCH_PRINT_MS)
    {
      lastFullSearchPrintMs = millis();
      Serial.print("NO_LINE_360 | searched=");
      Serial.print(fullSearchAccumulatedDeg, 1);
      Serial.print("/");
      Serial.print(FULL_SEARCH_TARGET_DEG, 0);
      Serial.print(" segment=");
      Serial.print(fullSearchSegmentAccumulatedDeg, 1);
      Serial.print("/");
      Serial.println(FULL_SEARCH_SEGMENT_DEG, 0);
    }
    return;
  }

  if (state == FIELD_NO_LINE_OPEN_PUSH)
  {
    if (lineRecovered())
    {
      Serial.println("NO_LINE | Line recovered during open push.");
      noLineSpinAttempts = 0;
      resetLinePid();
      enterState(FIELD_EXPLORE_LINE);
      return;
    }
    int16_t left = OPEN_PUSH_SPEED;
    int16_t right = OPEN_PUSH_SPEED;
    if (openBiasDirection < 0)
    {
      left -= 120;
      right += 120;
    }
    else if (openBiasDirection > 0)
    {
      left += 120;
      right -= 120;
    }
    driveBySide(left, right);
    if (millis() - stateStartMs >= OPEN_PUSH_MS)
    {
      stopAllMotors();
      enterState(FIELD_NO_LINE_SPIN);
    }
  }
}

bool initRFIDNoResetPin()
{
  Wire1.beginTransmission(RFID_I2C_ADDR);
  if (Wire1.endTransmission() != 0)
  {
    Serial.println("RFID | Not found at I2C address 0x28.");
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
  Serial.println(version, HEX);
  return version != 0x00 && version != 0xFF;
}

bool readRFIDCard(char *tagId, size_t tagIdSize)
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

  tagId[0] = '\0';
  size_t offset = 0;
  for (byte i = 0; i < mfrc522.uid.size && offset + 2 < tagIdSize; i++)
  {
    snprintf(tagId + offset, tagIdSize - offset, "%02X", mfrc522.uid.uidByte[i]);
    offset += 2;
  }
  mfrc522.PICC_HaltA();
  return tagId[0] != '\0';
}

bool sendServerText(const char *payload)
{
  if (!USE_SERVER_MESSENGER || !serverMessengerStarted || payload == nullptr)
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
  snprintf(payload, sizeof(payload), "type=register team_id=%s board_id=%s", GROUP_ID, BOARD_ID);
  return sendServerText(payload);
}

bool sendServerIsFertile(const char *tagId)
{
  char payload[144];
  snprintf(payload,
           sizeof(payload),
           "type=isFertile team_id=%s tag_id=%s board_id=%s",
           GROUP_ID,
           tagId,
           BOARD_ID);
  return sendServerText(payload);
}

bool sendServerSeedPlanted(const char *tagId)
{
  char payload[144];
  snprintf(payload,
           sizeof(payload),
           "type=seedPlanted team_id=%s tag_id=%s board_id=%s",
           GROUP_ID,
           tagId,
           BOARD_ID);
  return sendServerText(payload);
}

void updatePositionFromReply()
{
  if (!serverHaveCoord)
  {
    return;
  }
  currentX = serverReplyX;
  currentY = serverReplyY;
  positionKnown = true;
  Serial.print("LOCALIZE | x=");
  Serial.print(currentX);
  Serial.print(" y=");
  Serial.print(currentY);
  Serial.print(" heading=");
  Serial.println(heading);
}

void onServerMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length)
{
  char message[MiniMessenger::kMaxPayloadSize + 1];
  size_t copyLen = length > MiniMessenger::kMaxPayloadSize ? MiniMessenger::kMaxPayloadSize : length;
  memcpy(message, payload, copyLen);
  message[copyLen] = '\0';

  Serial.print("SERVER RX | from=");
  Serial.print(metadata.fromBoardId);
  Serial.print(" ");
  Serial.println(message);

  if (messageTypeEquals(message, "heartbeat"))
  {
    bool enabled = true;
    int timeLeft = -1;
    if (parseBoolField(message, "enable", enabled))
    {
      serverHeartbeatAllowsMovement = enabled;
    }
    if (parseIntField(message, "time_left", timeLeft))
    {
      serverTimeLeftSec = timeLeft;
    }
    return;
  }

  if (messageTypeEquals(message, "emergency"))
  {
    bool enabled = false;
    if (parseBoolField(message, "enabled", enabled))
    {
      serverEmergencyActive = enabled;
      if (enabled)
      {
        stopAllMotors();
      }
    }
    return;
  }

  if (messageTypeEquals(message, "disable"))
  {
    bool enabled = true;
    if (parseBoolField(message, "enabled", enabled))
    {
      serverDisableActive = !enabled;
      if (!enabled)
      {
        stopAllMotors();
      }
    }
    return;
  }

  if (messageTypeEquals(message, "isFertileReply"))
  {
    bool fertile = false;
    bool planted = false;
    int x = -1;
    int y = -1;
    serverFertile = parseBoolField(message, "fertile", fertile) && fertile;
    serverPlanted = parseBoolField(message, "planted", planted) && planted;
    serverHaveCoord = parseIntField(message, "x", x) && parseIntField(message, "y", y);
    if (serverHaveCoord)
    {
      serverReplyX = (int8_t)x;
      serverReplyY = (int8_t)y;
    }
    serverRfidReplyReceived = true;
    Serial.print("SERVER | fertility fertile=");
    Serial.print(serverFertile ? "true" : "false");
    Serial.print(" planted=");
    Serial.print(serverPlanted ? "true" : "false");
    if (serverHaveCoord)
    {
      Serial.print(" x=");
      Serial.print(serverReplyX);
      Serial.print(" y=");
      Serial.print(serverReplyY);
    }
    Serial.println();
  }
}

void setupServerMessenger()
{
  if (!USE_SERVER_MESSENGER)
  {
    Serial.println("SERVER | OFFLINE_TEST_MODE enabled. MiniMessenger disabled.");
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
  if (!USE_SERVER_MESSENGER || !serverMessengerStarted)
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
  if (connected && pendingSeedReport)
  {
    if (sendServerSeedPlanted(pendingSeedReportTagId))
    {
      pendingSeedReport = false;
      pendingSeedReportTagId[0] = '\0';
    }
  }
}

bool serverStopActive()
{
  return USE_SERVER_MESSENGER &&
         serverMessengerStarted &&
         (!serverHeartbeatAllowsMovement || serverDisableActive);
}

const char *planterStateName()
{
  switch (planterState)
  {
    case PLANTER_IDLE: return "IDLE";
    case PLANTER_SWEEP_FORWARD: return "SWEEP_FORWARD";
    case PLANTER_FORWARD_PAUSE: return "FORWARD_PAUSE";
    case PLANTER_SWEEP_BACKWARD: return "SWEEP_BACKWARD";
    case PLANTER_BACKWARD_PAUSE: return "BACKWARD_PAUSE";
  }
  return "UNKNOWN";
}

void setupSeedPlanter()
{
  planterServo.attach(SERVO_SIGNAL_PIN);
  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  planterState = PLANTER_IDLE;
  Serial.print("PLANTER | Servo D");
  Serial.print(SERVO_SIGNAL_PIN);
  Serial.println(" ready.");
}

void runStartupServoTest()
{
  Serial.println("SERVO_TEST | START full open/close sweep. This does not count as planting.");

  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  delay(400);

  for (int angle = PLANTER_MIN_ANGLE; angle <= PLANTER_MAX_ANGLE; angle++)
  {
    planterServo.write(angle);
    delay(PLANTER_STEP_DELAY_MS);
  }
  Serial.println("SERVO_TEST | OPEN reached.");
  delay(PLANTER_END_PAUSE_MS);

  for (int angle = PLANTER_MAX_ANGLE; angle >= PLANTER_MIN_ANGLE; angle--)
  {
    planterServo.write(angle);
    delay(PLANTER_STEP_DELAY_MS);
  }
  Serial.println("SERVO_TEST | CLOSED reached.");
  delay(PLANTER_END_PAUSE_MS);

  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  planterState = PLANTER_IDLE;
  seedReportArmed = false;
  plantingRFIDTagId[0] = '\0';
  Serial.println("SERVO_TEST | COMPLETE. Verify the mechanism moved; continuing startup.");
}

bool triggerSeedPlanter(const char *tagId)
{
  if (planterState != PLANTER_IDLE)
  {
    Serial.print("PLANTER | Busy ");
    Serial.println(planterStateName());
    return false;
  }
  if (lastPlantTriggerMs != 0 && millis() - lastPlantTriggerMs < RFID_PLANT_COOLDOWN_MS)
  {
    Serial.println("PLANTER | Cooldown; seed skipped.");
    return false;
  }
  copyText(plantingRFIDTagId, sizeof(plantingRFIDTagId), tagId);
  seedReportArmed = plantingRFIDTagId[0] != '\0' && USE_SERVER_MESSENGER;
  lastPlantTriggerMs = millis();
  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  planterLastStepMs = millis();
  planterState = PLANTER_SWEEP_FORWARD;
  Serial.print("PLANTER | Trigger tag_id=");
  Serial.println(tagId);
  return true;
}

void updateSeedPlanter()
{
  if (planterState == PLANTER_IDLE)
  {
    return;
  }
  unsigned long nowMs = millis();
  switch (planterState)
  {
    case PLANTER_SWEEP_FORWARD:
      if (nowMs - planterLastStepMs < PLANTER_STEP_DELAY_MS)
      {
        return;
      }
      planterLastStepMs = nowMs;
      planterAngle++;
      if (planterAngle >= PLANTER_MAX_ANGLE)
      {
        planterAngle = PLANTER_MAX_ANGLE;
        planterServo.write(planterAngle);
        planterPauseStartMs = nowMs;
        planterState = PLANTER_FORWARD_PAUSE;
        Serial.println("PLANTER | Open");
        return;
      }
      planterServo.write(planterAngle);
      break;

    case PLANTER_FORWARD_PAUSE:
      if (nowMs - planterPauseStartMs >= PLANTER_END_PAUSE_MS)
      {
        planterLastStepMs = nowMs;
        planterState = PLANTER_SWEEP_BACKWARD;
        Serial.println("PLANTER | Closing");
      }
      break;

    case PLANTER_SWEEP_BACKWARD:
      if (nowMs - planterLastStepMs < PLANTER_STEP_DELAY_MS)
      {
        return;
      }
      planterLastStepMs = nowMs;
      planterAngle--;
      if (planterAngle <= PLANTER_MIN_ANGLE)
      {
        planterAngle = PLANTER_MIN_ANGLE;
        planterServo.write(planterAngle);
        planterPauseStartMs = nowMs;
        planterState = PLANTER_BACKWARD_PAUSE;
        Serial.println("PLANTER | Closed");
        return;
      }
      planterServo.write(planterAngle);
      break;

    case PLANTER_BACKWARD_PAUSE:
      if (nowMs - planterPauseStartMs >= PLANTER_END_PAUSE_MS)
      {
        planterState = PLANTER_IDLE;
        seedsPlanted++;
        Serial.print("PLANTER | Done seeds=");
        Serial.println(seedsPlanted);
        if (seedReportArmed)
        {
          copyText(pendingSeedReportTagId, sizeof(pendingSeedReportTagId), plantingRFIDTagId);
          pendingSeedReport = pendingSeedReportTagId[0] != '\0';
          seedReportArmed = false;
          Serial.print("SERVER | Queued seedPlanted tag_id=");
          Serial.println(pendingSeedReportTagId);
        }
        if (seedsPlanted >= MAX_SEEDS)
        {
          Serial.println("MISSION | Max seeds planted. Request return to base.");
          enterState(FIELD_RETURN_REQUESTED);
        }
        else
        {
          enterState(FIELD_POST_NODE_DECIDE);
        }
      }
      break;

    case PLANTER_IDLE:
      break;
  }
}

void startRFIDQuery(const char *tagId)
{
  stopAllMotors();
  resetLinePid();
  copyText(activeRFIDTagId, sizeof(activeRFIDTagId), tagId);
  nodesScanned++;
  serverRfidWaiting = false;
  serverRfidReplyReceived = false;
  serverFertile = false;
  serverPlanted = false;
  serverHaveCoord = false;
  Serial.print("RFID | tag_id=");
  Serial.print(activeRFIDTagId);

  if (OFFLINE_TEST_MODE)
  {
    serverFertile = (nodesScanned % 2) == 1;
    serverPlanted = false;
    serverHaveCoord = false;
    serverRfidReplyReceived = true;
    Serial.println(" offline fake reply.");
  }
  else if (USE_SERVER_MESSENGER && serverMessengerStarted && serverMessenger.isConnected())
  {
    serverRfidWaiting = true;
    serverRfidRequestMs = millis();
    Serial.println(" query fertile.");
    if (!sendServerIsFertile(activeRFIDTagId))
    {
      serverRfidWaiting = false;
    }
  }
  else
  {
    Serial.println(" server not connected; no planting.");
  }
  lastRFIDActionMs = millis();
  enterState(FIELD_RFID_WAIT);
}

int8_t recentRFIDSlot(const char *tagId)
{
  for (uint8_t i = 0; i < RFID_RECENT_NODE_COUNT; i++)
  {
    if (recentRFIDTagIds[i][0] != '\0' && strcmp(tagId, recentRFIDTagIds[i]) == 0)
    {
      return (int8_t)i;
    }
  }
  return -1;
}

void rememberRFIDNode(const char *tagId)
{
  int8_t existingSlot = recentRFIDSlot(tagId);
  uint8_t slot = existingSlot >= 0 ? (uint8_t)existingSlot : nextRecentRFIDSlot;
  copyText(recentRFIDTagIds[slot], sizeof(recentRFIDTagIds[slot]), tagId);
  recentRFIDSeenMs[slot] = millis();
  if (existingSlot < 0)
  {
    nextRecentRFIDSlot = (nextRecentRFIDSlot + 1) % RFID_RECENT_NODE_COUNT;
  }
}

void updateRFID()
{
  if (!rfidReady)
  {
    if (millis() - lastRFIDRetryMs >= RFID_RETRY_INTERVAL_MS)
    {
      lastRFIDRetryMs = millis();
      rfidReady = initRFIDNoResetPin();
    }
    return;
  }

  if (millis() - lastRFIDCheckMs < RFID_CHECK_INTERVAL_MS)
  {
    return;
  }
  lastRFIDCheckMs = millis();

  if (millis() - lastRFIDActionMs < RFID_CARD_COOLDOWN_MS)
  {
    return;
  }
  if (state == FIELD_RFID_WAIT ||
      state == FIELD_PRE_PLANT_FORWARD ||
      state == FIELD_PLANTING_WAIT ||
      state == FIELD_TURNING ||
      state == FIELD_NO_LINE_BACKUP ||
      state == FIELD_NO_LINE_SPIN ||
      state == FIELD_NO_LINE_FULL_SPIN ||
      state == FIELD_NO_LINE_OPEN_PUSH ||
      state == FIELD_RETURN_REQUESTED ||
      state == FIELD_RETURN_TUNNEL)
  {
    return;
  }

  char tagId[RFID_TAG_ID_MAX_LEN];
  if (!readRFIDCard(tagId, sizeof(tagId)))
  {
    if (lastAcceptedRFIDTagId[0] != '\0')
    {
      if (rfidAbsentSinceMs == 0)
      {
        rfidAbsentSinceMs = millis();
      }
      else if (millis() - rfidAbsentSinceMs >= RFID_ABSENT_CONFIRM_MS)
      {
        rfidLeftLastAcceptedTag = true;
      }
    }
    return;
  }

  rfidAbsentSinceMs = 0;
  bool sameAsLastAccepted = strcmp(tagId, lastAcceptedRFIDTagId) == 0;
  int8_t recentSlot = recentRFIDSlot(tagId);
  bool recentNodeCooldownActive =
      recentSlot >= 0 &&
      millis() - recentRFIDSeenMs[(uint8_t)recentSlot] < RFID_NODE_REVISIT_COOLDOWN_MS;
  if (sameAsLastAccepted || recentNodeCooldownActive)
  {
    if (millis() - lastRFIDDuplicatePrintMs >= RFID_DUPLICATE_PRINT_MS)
    {
      lastRFIDDuplicatePrintMs = millis();
      Serial.print("RFID | recent node ignored tag_id=");
      Serial.print(tagId);
      if (sameAsLastAccepted)
      {
        Serial.println(" waitingForDifferentNode=YES");
      }
      else
      {
        Serial.print(" cooldownInMs=");
        unsigned long elapsed = millis() - recentRFIDSeenMs[(uint8_t)recentSlot];
        Serial.println(RFID_NODE_REVISIT_COOLDOWN_MS - elapsed);
      }
    }
    return;
  }

  if (tagId[0] != '\0')
  {
    copyText(lastAcceptedRFIDTagId, sizeof(lastAcceptedRFIDTagId), tagId);
    rfidLeftLastAcceptedTag = false;
    rememberRFIDNode(tagId);
    copyText(lastRFIDTagId, sizeof(lastRFIDTagId), tagId);
    startRFIDQuery(tagId);
  }
}

void handleRFIDWait()
{
  stopAllMotors();
  bool timedOut = serverRfidWaiting &&
                  !serverRfidReplyReceived &&
                  millis() - serverRfidRequestMs >= SERVER_REPLY_TIMEOUT_MS;
  if (serverRfidWaiting && !serverRfidReplyReceived && !timedOut)
  {
    return;
  }
  if (timedOut)
  {
    serverRfidWaiting = false;
    serverRfidReplyReceived = true;
    Serial.println("SERVER | Fertility timeout. No seed.");
  }
  if (millis() - stateStartMs < RFID_PAUSE_MS)
  {
    return;
  }

  updatePositionFromReply();
  if (returnModeActive)
  {
    serverRfidWaiting = false;
    Serial.println("RETURN | RFID coordinate received; skip planting and replan.");
    planReturnStep();
    return;
  }
  if (serverFertile)
  {
    fertileSeen++;
  }
  else
  {
    infertileSeen++;
  }

  bool desperatePlant =
      ALLOW_DESPERATE_INFERTILE_PLANTING &&
      !serverFertile &&
      seedsPlanted < 2 &&
      ((serverTimeLeftSec >= 0 && serverTimeLeftSec <= FORCE_RETURN_TIME_LEFT_SEC + 15) ||
       (millis() - runStartMs >= FORCE_RETURN_ELAPSED_MS - 15000UL));
  bool shouldPlant = seedsPlanted < MAX_SEEDS &&
                     !serverPlanted &&
                     (serverFertile || desperatePlant);

  Serial.print("RFID | decision fertile=");
  Serial.print(serverFertile ? "true" : "false");
  Serial.print(" planted=");
  Serial.print(serverPlanted ? "true" : "false");
  Serial.print(" desperate=");
  Serial.print(desperatePlant ? "true" : "false");
  Serial.print(" shouldPlant=");
  Serial.println(shouldPlant ? "true" : "false");

  serverRfidWaiting = false;
  if (shouldPlant)
  {
    Serial.print("PLANTER | Move forward about 1.3cm before planting tag_id=");
    Serial.println(activeRFIDTagId);
    enterState(FIELD_PRE_PLANT_FORWARD);
  }
  else
  {
    Serial.print("PLANTER | skip reason=");
    if (serverPlanted)
    {
      Serial.println("already_planted");
    }
    else if (!serverFertile && !desperatePlant)
    {
      Serial.println("infertile");
    }
    else if (seedsPlanted >= MAX_SEEDS)
    {
      Serial.println("seed_limit");
    }
    else
    {
      Serial.println("not_eligible");
    }
    enterState(FIELD_POST_NODE_DECIDE);
  }
}

void handlePostNodeDecide()
{
  if (millis() - stateStartMs < 250)
  {
    stopAllMotors();
    return;
  }
  if (seedsPlanted >= MAX_SEEDS)
  {
    enterState(FIELD_RETURN_REQUESTED);
    return;
  }

  readSensors();
  if (!anySensorOnLine())
  {
    Serial.println("EXPLORE | No line after node. Recover before choosing direction.");
    startOpenSpaceRecovery("post-node line absent");
    return;
  }

  bool leftBranch = false;
  bool rightBranch = false;
  branchSeen(leftBranch, rightBranch);
  bool straightAvailable = sensorOnLine[SENSOR_MIDDLE] ||
                           sensorOnLine[SENSOR_SLIGHT_LEFT] ||
                           sensorOnLine[SENSOR_SLIGHT_RIGHT];
  int decision = chooseExploreTurn(leftBranch, rightBranch, straightAvailable);
  if (decision != 0)
  {
    beginTurn(decision, "post-node confirmed branch");
    return;
  }

  Serial.println("EXPLORE | post-node resume PID line following");
  branchCooldownUntilMs = millis() + BRANCH_COOLDOWN_MS;
  resetLinePid();
  enterState(FIELD_EXPLORE_LINE);
}

void printSensorsAndStatus()
{
  if (millis() - lastStatusMs < STATUS_INTERVAL_MS)
  {
    return;
  }
  lastStatusMs = millis();
  Serial.print("STATUS | State=");
  Serial.print(stateName(state));
  Serial.print(" Seeds=");
  Serial.print(seedsPlanted);
  Serial.print(" Nodes=");
  Serial.print(nodesScanned);
  Serial.print(" Fertile=");
  Serial.print(fertileSeen);
  Serial.print(" Infertile=");
  Serial.print(infertileSeen);
  Serial.print(" Pos=");
  if (positionKnown)
  {
    Serial.print(currentX);
    Serial.print(",");
    Serial.print(currentY);
  }
  else
  {
    Serial.print("?");
  }
  Serial.print(" Heading=");
  Serial.print(heading);
  Serial.print(" Return=");
  Serial.print(returnModeActive ? "YES" : "NO");
  Serial.print(" TimeLeft=");
  Serial.print(serverTimeLeftSec);
  Serial.print(" US F=");
  Serial.print(usFrontCm, 1);
  Serial.print(" L=");
  Serial.print(usLeftCm, 1);
  Serial.print(" R=");
  Serial.print(usRightCm, 1);
  Serial.print(" Yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" NoLineMs=");
  Serial.print(noLineEpisodeStartMs == 0 ? 0 : millis() - noLineEpisodeStartMs);
  Serial.print(" | QTR ");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print("=");
    Serial.print(sensorValues[i]);
    Serial.print(sensorOnLine[i] ? "* " : " ");
  }
  Serial.println();
}

void handleExploreLine()
{
  readSensors();
  printSensorsAndStatus();

  if (frontObstacleDetected())
  {
    if (ENABLE_REVIVAL && goodMiddleAreaForRevival())
    {
      Serial.println("REVIVAL | Front robot/obstacle in open middle. Start controlled bump.");
      enterState(FIELD_REVIVAL_APPROACH);
      return;
    }
    int direction = chooseOpenDirection();
    Serial.print("OBSTACLE | Front obstacle. Avoid toward ");
    Serial.println(direction < 0 ? "LEFT/open" : "RIGHT/open");
    beginTurn(direction, "front obstacle open-side");
    return;
  }

  if (!anySensorOnLine())
  {
    if (returnModeActive && positionKnown && currentY > 4)
    {
      if (noLineEpisodeStartMs == 0)
      {
        noLineEpisodeStartMs = millis();
        Serial.println("RETURN | Open field: hold IMU heading until next RFID node.");
      }
      if (noLineSelfRescueDue())
      {
        startNoLineSelfRescue("Return open-field motion stalled without line.");
        return;
      }
      float targetYaw = (float)heading * 90.0;
      float yawError = normalizedAngleDeltaDeg(yawDeg, targetYaw);
      int16_t correction = (int16_t)constrain(RETURN_YAW_KP * yawError, -110.0f, 110.0f);
      driveBySide(RETURN_OPEN_SPEED - correction, RETURN_OPEN_SPEED + correction);
      return;
    }
    lineRecoveryCandidateSinceMs = 0;
    if (noLineEpisodeStartMs == 0)
    {
      noLineEpisodeStartMs = millis();
      Serial.println("NO_LINE_TIMER | Started. Self rescue after 10000ms without stable line.");
    }
    if (noLineSelfRescueDue())
    {
      startNoLineSelfRescue("Intermittent/absent line persisted for 10000ms.");
      return;
    }
    if (lineLostSinceMs == 0)
    {
      lineLostSinceMs = millis();
      lineGapPrinted = false;
    }
    if (millis() - lineLostSinceMs < LINE_LOST_CONFIRM_MS)
    {
      if (!lineGapPrinted)
      {
        lineGapPrinted = true;
        Serial.println("LINE_GAP | transient all-white; crossing straight before recovery.");
      }
      driveForward(LINE_GAP_CROSS_SPEED);
      return;
    }
    lineLostSinceMs = 0;
    lineGapPrinted = false;
    startOpenSpaceRecovery("line lost");
    return;
  }

  if (lineLostSinceMs != 0)
  {
    Serial.println("LINE_GAP | line returned; no recovery needed.");
    lineLostSinceMs = 0;
    lineGapPrinted = false;
    resetLinePid();
  }

  if (noLineEpisodeStartMs != 0)
  {
    if (lineRecoveryCandidateSinceMs == 0)
    {
      lineRecoveryCandidateSinceMs = millis();
      Serial.println("NO_LINE_TIMER | Line returned; waiting for stable confirmation.");
    }
    else if (millis() - lineRecoveryCandidateSinceMs >= LINE_RECOVERY_STABLE_MS)
    {
      noLineEpisodeStartMs = 0;
      lineRecoveryCandidateSinceMs = 0;
      Serial.println("NO_LINE_TIMER | Stable line confirmed; timer cleared.");
    }
  }

  if (millis() >= branchCooldownUntilMs && millis() >= postTurnIgnoreUntilMs)
  {
    bool leftBranch = false;
    bool rightBranch = false;
    if (branchSeen(leftBranch, rightBranch))
    {
      bool straightAvailable = sensorOnLine[SENSOR_MIDDLE] ||
                               sensorOnLine[SENSOR_SLIGHT_LEFT] ||
                               sensorOnLine[SENSOR_SLIGHT_RIGHT];
      int decision = chooseExploreTurn(leftBranch, rightBranch, straightAvailable);
      Serial.print("BRANCH | left=");
      Serial.print(leftBranch ? "Y" : "N");
      Serial.print(" right=");
      Serial.print(rightBranch ? "Y" : "N");
      Serial.print(" straight=");
      Serial.print(straightAvailable ? "Y" : "N");
      Serial.print(" decision=");
      Serial.println(decision);
      if (decision != 0)
      {
        beginTurn(decision, "open-weighted branch");
        return;
      }
      branchCooldownUntilMs = millis() + BRANCH_COOLDOWN_MS;
    }
  }

  float error = 0.0;
  if (!computeLinePidError(error))
  {
    startOpenSpaceRecovery("pid signal weak");
    return;
  }
  driveLinePid(error);
}

void planReturnStep()
{
  returnModeActive = true;
  stopAllMotors();
  noLineEpisodeStartMs = 0;
  lineRecoveryCandidateSinceMs = 0;
  lineLostSinceMs = 0;
  if (!positionKnown)
  {
    Serial.println("RETURN | Position unknown. Continue cautiously until next RFID coordinate.");
    enterState(FIELD_EXPLORE_LINE);
    return;
  }

  if (currentX == RETURN_EXIT_X && currentY == RETURN_EXIT_Y)
  {
    Serial.println("RETURN | Exit (9,7) reached. Enter tunnel with ultrasonic centering.");
    enterState(FIELD_RETURN_TUNNEL);
    return;
  }

  int8_t desiredHeading = heading;
  if (currentY < RETURN_EXIT_Y)
  {
    desiredHeading = 0;
  }
  else if (currentY > RETURN_EXIT_Y)
  {
    desiredHeading = 2;
  }
  else if (currentX < RETURN_EXIT_X)
  {
    desiredHeading = 1;
  }
  else if (currentX > RETURN_EXIT_X)
  {
    desiredHeading = 3;
  }

  uint8_t delta = (desiredHeading - heading + 4) % 4;
  Serial.print("RETURN | plan pos=");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.print(" heading=");
  Serial.print(heading);
  Serial.print(" desired=");
  Serial.println(desiredHeading);
  if (delta == 0)
  {
    enterState(FIELD_EXPLORE_LINE);
    return;
  }

  returnQuarterTurnsPending = delta == 2 ? 1 : 0;
  returnPendingTurnDirection = delta == 3 ? -1 : (delta == 1 ? 1 : chooseOpenDirection());
  beginTurn(returnPendingTurnDirection, delta == 2 ? "return U-turn first quarter" : "return route align");
}

void handleReturnTunnel()
{
  printSensorsAndStatus();
  if (millis() - stateStartMs >= RETURN_TUNNEL_MS)
  {
    Serial.println("RETURN | Tunnel traversal complete. Resume line tracking inside base.");
    returnModeActive = false;
    reviveLedGreen = true;
    enterState(FIELD_EXPLORE_LINE);
    return;
  }

  if (usFrontCm > 0.0 && usFrontCm < 8.0)
  {
    pivotRight(TURN_FINE_SPEED);
    return;
  }

  int16_t correction = 0;
  bool leftValid = usLeftCm > 0.0 && usLeftCm < 100.0;
  bool rightValid = usRightCm > 0.0 && usRightCm < 100.0;
  if (leftValid && rightValid)
  {
    correction = (int16_t)constrain((usRightCm - usLeftCm) * RETURN_TUNNEL_KP,
                                    (float)-RETURN_TUNNEL_MAX_CORRECTION,
                                    (float)RETURN_TUNNEL_MAX_CORRECTION);
  }
  else if (leftValid && usLeftCm < RETURN_TUNNEL_CRITICAL_CM)
  {
    correction = RETURN_TUNNEL_MAX_CORRECTION;
  }
  else if (rightValid && usRightCm < RETURN_TUNNEL_CRITICAL_CM)
  {
    correction = -RETURN_TUNNEL_MAX_CORRECTION;
  }
  driveBySide(max((int16_t)(RETURN_TUNNEL_SPEED + correction), RETURN_TUNNEL_MIN_SPEED),
              max((int16_t)(RETURN_TUNNEL_SPEED - correction), RETURN_TUNNEL_MIN_SPEED));
}

bool forcedReturnRequired()
{
  if (serverEmergencyActive)
  {
    return true;
  }
  if (serverTimeLeftSec >= 0 && serverTimeLeftSec <= FORCE_RETURN_TIME_LEFT_SEC)
  {
    return true;
  }
  return runStartMs != 0 && millis() - runStartMs >= FORCE_RETURN_ELAPSED_MS;
}

void startReturnRequested(const char *reason)
{
  stopAllMotors();
  returnModeActive = true;
  Serial.print("RETURN | requested reason=");
  Serial.print(reason);
  Serial.print(" pos=");
  if (positionKnown)
  {
    Serial.print(currentX);
    Serial.print(",");
    Serial.print(currentY);
  }
  else
  {
    Serial.print("unknown");
  }
  Serial.print(" seeds=");
  Serial.println(seedsPlanted);
  enterState(FIELD_RETURN_REQUESTED);
}

void updateStuckMonitor()
{
  bool motorCommanded = false;
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    if (abs(lastMotorCommand[i]) >= 180)
    {
      motorCommanded = true;
      break;
    }
  }
  if (!motorCommanded ||
      state == FIELD_RFID_WAIT ||
      state == FIELD_PLANTING_WAIT ||
      state == FIELD_RETURN_REQUESTED)
  {
    stuckSinceMs = 0;
    copyEncoderCounts(stuckSampleEncoder);
    stuckSampleYaw = yawDeg;
    stuckSampleMs = millis();
    return;
  }

  unsigned long now = millis();
  if (stuckSampleMs == 0 || now - stuckSampleMs < STUCK_SAMPLE_MS)
  {
    return;
  }

  long motion = encoderMotionSince(stuckSampleEncoder);
  float yawMotion = fabs(normalizedAngleDeltaDeg(yawDeg, stuckSampleYaw));

  long wheelDelta[ENCODER_COUNT];
  uint8_t activeWheels = 0;
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    wheelDelta[i] = labs(encoderCounts[i] - stuckSampleEncoder[i]);
    if (wheelDelta[i] > 0)
    {
      activeWheels++;
    }
  }
  if (activeWheels > 0 && activeWheels < 3)
  {
    Serial.print("STUCK_WARN | odd encoder active=");
    Serial.print(activeWheels);
    Serial.print(" d=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(wheelDelta[i]);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.println();
  }

  bool noMotion = motion <= STUCK_MIN_ENCODER_TICKS && yawMotion <= STUCK_MIN_YAW_DELTA_DEG;
  if (noMotion)
  {
    if (stuckSinceMs == 0)
    {
      stuckSinceMs = now;
    }
    else if (now - stuckSinceMs >= STUCK_CONFIRM_MS)
    {
      Serial.print("STUCK | confirmed motion=");
      Serial.print(motion);
      Serial.print(" yawDelta=");
      Serial.print(yawMotion, 2);
      Serial.println(" -> self rescue");
      openBiasDirection = chooseOpenDirection();
      copyEncoderCounts(rescueStartEncoder);
      enterState(FIELD_SELF_RESCUE_BACKUP);
      stuckSinceMs = 0;
    }
  }
  else
  {
    stuckSinceMs = 0;
  }

  copyEncoderCounts(stuckSampleEncoder);
  stuckSampleYaw = yawDeg;
  stuckSampleMs = now;
}

void handleSelfRescue()
{
  if (millis() - lastStatusMs >= STATUS_INTERVAL_MS)
  {
    readSensors();
    printSensorsAndStatus();
  }

  if (state == FIELD_SELF_RESCUE_BACKUP)
  {
    driveBackwardWithOpenBias(LOST_BACKUP_SPEED);
    if (encoderMotionSince(rescueStartEncoder) >= NODE_BACKUP_ENCODER_TICKS / 2 ||
        millis() - stateStartMs >= 1000)
    {
      stopAllMotors();
      enterState(FIELD_SELF_RESCUE_SPIN);
    }
    return;
  }

  if (state == FIELD_SELF_RESCUE_SPIN)
  {
    if (lineRecovered())
    {
      Serial.println("SELF_RESCUE | Line found.");
      resetLinePid();
      enterState(FIELD_EXPLORE_LINE);
      return;
    }
    if (openBiasDirection < 0)
    {
      pivotLeft(LOST_SPIN_SPEED);
    }
    else
    {
      pivotRight(LOST_SPIN_SPEED);
    }
    if (millis() - stateStartMs >= LOST_SPIN_MS + 1200)
    {
      stopAllMotors();
      enterState(FIELD_SELF_RESCUE_OPEN_PUSH);
    }
    return;
  }

  if (state == FIELD_SELF_RESCUE_OPEN_PUSH)
  {
    int16_t left = SELF_RESCUE_PUSH_SPEED;
    int16_t right = SELF_RESCUE_PUSH_SPEED;
    if (openBiasDirection < 0)
    {
      left -= 180;
      right += 120;
    }
    else
    {
      left += 120;
      right -= 180;
    }
    driveBySide(left, right);
    if (lineRecovered())
    {
      Serial.println("SELF_RESCUE | Line found during high-power open push.");
      resetLinePid();
      enterState(FIELD_EXPLORE_LINE);
      return;
    }
    if (millis() - stateStartMs >= 900)
    {
      stopAllMotors();
      enterState(FIELD_NO_LINE_SPIN);
    }
  }
}

void handleRevival()
{
  if (state == FIELD_REVIVAL_APPROACH)
  {
    driveForward(REVIVAL_APPROACH_SPEED);
    if ((validDistance(usFrontCm) && usFrontCm <= REVIVAL_CONTACT_CM) ||
        millis() - stateStartMs >= REVIVAL_APPROACH_TIMEOUT_MS)
    {
      stopAllMotors();
      Serial.println("REVIVAL | Contact attempt complete. Waiting 7s.");
      enterState(FIELD_REVIVAL_WAIT);
    }
    return;
  }

  if (state == FIELD_REVIVAL_WAIT)
  {
    stopAllMotors();
    if (millis() - stateStartMs >= REVIVAL_WAIT_MS)
    {
      copyEncoderCounts(rescueStartEncoder);
      Serial.println("REVIVAL | Backing one node, then resume exploration.");
      enterState(FIELD_REVIVAL_BACKUP_NODE);
    }
    return;
  }

  if (state == FIELD_REVIVAL_BACKUP_NODE)
  {
    driveForward(-REVIVAL_BACKUP_SPEED);
    if (encoderMotionSince(rescueStartEncoder) >= NODE_BACKUP_ENCODER_TICKS ||
        millis() - stateStartMs >= 1800)
    {
      stopAllMotors();
      enterState(FIELD_EXPLORE_LINE);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  randomSeed(micros());

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  beginButton(reviveButton);
  beginButton(killButton);
  beginEncoders();
  ignoreD45UntilReleased = digitalRead(KILL_BUTTON_PIN) == LOW;

  setupSeedPlanter();
  if (ENABLE_STARTUP_SERVO_TEST)
  {
    runStartupServoTest();
  }
  else
  {
    Serial.println("SERVO_TEST | Disabled. Set ENABLE_STARTUP_SERVO_TEST=true to enable.");
  }

  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
  digitalWrite(US_FRONT_TRIG, LOW);
  digitalWrite(US_LEFT_TRIG, LOW);
  digitalWrite(US_RIGHT_TRIG, LOW);

  Wire.begin();
  Wire.setClock(400000);
  while (!initIMU())
  {
    Serial.println("IMU | Not ready, retrying...");
    delay(500);
  }

  Wire1.begin();
  Wire1.setClock(400000);
  setupMotoron(mcLower, RF, RR, LR);
  setupMotoron(mcUpper, LF);
  stopAllMotors();

  rfidReady = initRFIDNoResetPin();
  setupServerMessenger();

  delay(800);
  calibrateGyroZ();
  yawDeg = 0.0;
  turnStartYawDeg = 0.0;
  lastUpdateMicros = micros();

  startupWaiting = true;
  killStopped = true;
  enterState(FIELD_EXPLORE_LINE);

  Serial.println("PRE_FIELD | Field planting controller ready.");
  Serial.print("PRE_FIELD | SECOND_ATTEMPT_MODE=");
  Serial.print(SECOND_ATTEMPT_MODE ? "true" : "false");
  Serial.print(" OFFLINE_TEST_MODE=");
  Serial.print(OFFLINE_TEST_MODE ? "true" : "false");
  Serial.print(" RFID=");
  Serial.println(rfidReady ? "ready" : "not ready");
  Serial.println("PRE_FIELD | D45 starts run. First attempt is stability-first; revive only in second attempt.");
  Serial.println("PRE_FIELD | Return triggers: 5 seeds, emergency, time_left<=75s, or 3:45 elapsed.");
  Serial.println("PRE_FIELD | No stable line for 10s escalates to self rescue; IMU search stalls no longer stop permanently.");
}

void loop()
{
  updateButtons();
  updateLed();
  pollEncoders();
  updateYaw();
  updateUltrasonic();
  updateServerMessenger();
  updateSeedPlanter();

  if (startupWaiting || killStopped)
  {
    stopAllMotors();
    delay(3);
    return;
  }

  if (serverStopActive())
  {
    stopAllMotors();
    if (millis() - serverLastStatusMs >= SERVER_STATUS_INTERVAL_MS)
    {
      serverLastStatusMs = millis();
      Serial.print("SERVER | Movement held heartbeat=");
      Serial.print(serverHeartbeatAllowsMovement ? "OK" : "STOP");
      Serial.print(" disable=");
      Serial.println(serverDisableActive ? "ON" : "OFF");
    }
    delay(3);
    return;
  }

  if (!returnModeActive && state != FIELD_RETURN_REQUESTED && forcedReturnRequired())
  {
    startReturnRequested(serverEmergencyActive ? "emergency" : "time");
  }

  updateRFID();
  updateStuckMonitor();

  switch (state)
  {
    case FIELD_EXPLORE_LINE:
      handleExploreLine();
      break;

    case FIELD_TURNING:
      handleTurning();
      break;

    case FIELD_RFID_WAIT:
      handleRFIDWait();
      break;

    case FIELD_PRE_PLANT_FORWARD:
      if (millis() - stateStartMs < PLANT_APPROACH_MS)
      {
        driveForward(PLANT_APPROACH_SPEED);
      }
      else
      {
        stopAllMotors();
        if (triggerSeedPlanter(activeRFIDTagId))
        {
          enterState(FIELD_PLANTING_WAIT);
        }
        else
        {
          Serial.println("PLANTER | Could not start after approach. Continue exploration.");
          enterState(FIELD_POST_NODE_DECIDE);
        }
      }
      break;

    case FIELD_PLANTING_WAIT:
      stopAllMotors();
      if (planterState == PLANTER_IDLE)
      {
        enterState(FIELD_POST_NODE_DECIDE);
      }
      break;

    case FIELD_POST_NODE_DECIDE:
      handlePostNodeDecide();
      break;

    case FIELD_NO_LINE_BACKUP:
    case FIELD_NO_LINE_SPIN:
    case FIELD_NO_LINE_FULL_SPIN:
    case FIELD_NO_LINE_OPEN_PUSH:
      handleNoLineRecovery();
      break;

    case FIELD_REVIVAL_APPROACH:
    case FIELD_REVIVAL_WAIT:
    case FIELD_REVIVAL_BACKUP_NODE:
      handleRevival();
      break;

    case FIELD_SELF_RESCUE_BACKUP:
    case FIELD_SELF_RESCUE_SPIN:
    case FIELD_SELF_RESCUE_OPEN_PUSH:
      handleSelfRescue();
      break;

    case FIELD_RETURN_REQUESTED:
      planReturnStep();
      break;

    case FIELD_RETURN_TUNNEL:
      handleReturnTunnel();
      break;

    case FIELD_DONE:
      stopAllMotors();
      break;
  }

  delay(2);
}
