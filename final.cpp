#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Servo.h>
#include "ICM_20948.h"
#include "secrets.h"

const bool SERVO_TEST = false;
const bool USE_SEED_PLANTER = true;
const bool OFFLINE_MODE = false; // true = no WiFi/server; RFID uses local fallback.
const bool USE_SERVER_MESSENGER = !OFFLINE_MODE;
const bool ALLOW_LOCAL_RFID_FALLBACK = OFFLINE_MODE;
const bool RFID_ONLY_TEST = false;
const bool MOTOR_FULL_SPEED_TEST = false;
const bool LF_MOTOR_ENCODER_TEST = false;

// Physical stack order is reversed on the rebuilt robot:
// the three-motor shield is now at 0x11, and the one-motor shield is at 0x10.
const uint8_t LOWER_MOTORON_ADDR = 0x11;
const uint8_t UPPER_MOTORON_ADDR = 0x10;

MotoronI2C mcLower(LOWER_MOTORON_ADDR);
MotoronI2C mcUpper(UPPER_MOTORON_ADDR);

// RFID WS1850S / MFRC522-compatible module on SDA1/SCL1.
const uint8_t RFID_I2C_ADDR = 0x28;
const uint8_t RFID_UNUSED_RESET_PIN = 0xFF;
MFRC522_I2C mfrc522(RFID_I2C_ADDR, RFID_UNUSED_RESET_PIN, &Wire1);
MiniMessenger serverMessenger;

// ICM-20948 on SDA/SCL, D20/D21.
ICM_20948_I2C imu;
bool imuAd0Val = true;
float gyroZBias = 0.0;
float yawDeg = 0.0;
float turnStartYawDeg = 0.0;
float pitchDeg = 0.0;
float rollDeg = 0.0;
float tiltBaselinePitchDeg = 0.0;
float tiltBaselineRollDeg = 0.0;
bool tiltBaselineCaptured = false;
unsigned long lastUpdateMicros = 0;

// Lower shield: RF/RR/LR on M1/M2/M3.
// Upper shield: LF on M1.
const uint8_t RF = 1;
const uint8_t RR = 2;
const uint8_t LR = 3;
const uint8_t LF = 1;

// Raw Motoron polarity for physical forward motion.
const int8_t FWD_RF = 1;
const int8_t FWD_RR = -1;
const int8_t FWD_LR = 1;
const int8_t FWD_LF = 1;

// Reflectance board using only OUT1/OUT3/OUT5/OUT7/OUT9, right to left.
// OUT2/OUT4/OUT6/OUT8 are intentionally not read.
const uint8_t SENSOR_COUNT = 5;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {22, 24, 26, 28, 31};
const char *SENSOR_NAMES[SENSOR_COUNT] = {"O1", "O3", "O5", "O7", "O9"};
const uint8_t SENSOR_RIGHT = 0;          // OUT1
const uint8_t SENSOR_SLIGHT_RIGHT = 1;   // OUT3
const uint8_t SENSOR_MIDDLE = 2;         // OUT5
const uint8_t SENSOR_SLIGHT_LEFT = 3;    // OUT7
const uint8_t SENSOR_LEFT = 4;           // OUT9
const uint8_t LOGICAL_SENSOR_COUNT = 5;
const uint8_t LOGICAL_SENSOR_INDEXES[LOGICAL_SENSOR_COUNT] = {
    SENSOR_RIGHT,
    SENSOR_SLIGHT_RIGHT,
    SENSOR_MIDDLE,
    SENSOR_SLIGHT_LEFT,
    SENSOR_LEFT,
};

// Encoder order is RF, RR, LR, LF.
const uint8_t ENCODER_COUNT = 4;
const uint8_t ENC_A[ENCODER_COUNT] = {18, 2, 4, 8};
const uint8_t ENC_B[ENCODER_COUNT] = {19, 3, 5, 9};
const uint8_t US_FRONT_ECHO = 37;
const uint8_t US_FRONT_TRIG = 39;
const uint8_t US_LEFT_ECHO = 52;
const uint8_t US_LEFT_TRIG = 44;
const uint8_t US_RIGHT_ECHO = 46;
const uint8_t US_RIGHT_TRIG = 48;
const uint8_t REVIVE_BUTTON_PIN = 32;
const uint8_t KILL_BUTTON_PIN = 45;
const uint8_t LED_RED_PIN = 36;
const uint8_t LED_GREEN_PIN = 38;
const uint8_t SERVO_SIGNAL_PIN = 33;

// Seed planter servo uses the same sweep as the standalone D9 test sketch:
// 0 -> 135 degrees, pause, then 135 -> 0 degrees.
const int PLANTER_MIN_ANGLE = 0;
const int PLANTER_MAX_ANGLE = 135;
const unsigned long PLANTER_STEP_DELAY_MS = 15;
const unsigned long PLANTER_END_PAUSE_MS = 500;
const unsigned long RFID_PLANT_COOLDOWN_MS = 3000;
const unsigned long RFID_PAUSE_MS = 1000;
const unsigned long RFID_CARD_ACTION_COOLDOWN_MS = 2500;
const unsigned long RFID_SAME_CARD_ACTION_COOLDOWN_MS = 30000;
const unsigned long SERVER_REGISTER_INTERVAL_MS = 10000;
const unsigned long SERVER_REPLY_TIMEOUT_MS = 1800;
const unsigned long SERVER_STATUS_INTERVAL_MS = 2000;
const size_t RFID_TAG_ID_MAX_LEN = 24;
const char *SERVER_TARGET_BOARD = "server";
const char *SERVER_AIRLOCK_OPEN_TYPE = "openAirlock";
const char *SERVER_AIRLOCK_ID = "A";
const int8_t RETURN_EXIT_X = 9;
const int8_t RETURN_EXIT_Y = 7;
const float OBSTACLE_TURN_DISTANCE_CM = 10.0;
const uint16_t FRONT_OBSTACLE_RECOVERY_MS = 1200;
const uint16_t LINE_LOST_STOP_RECOVERY_MS = 500;
const bool USE_GRID_OBSTACLE_AVOIDANCE = false;
const float GRID_OBSTACLE_DETECT_CM = 15.0;
const int16_t GRID_OBSTACLE_FORWARD_SPEED = 650;
const int16_t GRID_OBSTACLE_TURN_FAST_SPEED = 760;
const int16_t GRID_OBSTACLE_TURN_FINE_SPEED = 620;
const float GRID_OBSTACLE_TURN_TARGET_DEG = 90.0;
const float GRID_OBSTACLE_TURN_TOLERANCE_DEG = 2.0;
const float GRID_OBSTACLE_TURN_FINE_REMAINING_DEG = 12.0;
const uint16_t GRID_OBSTACLE_TURN_SETTLE_MS = 160;
const uint16_t GRID_OBSTACLE_TURN_TIMEOUT_MS = 7000;
const uint16_t GRID_OBSTACLE_STRAIGHT_TIMEOUT_MS = 11000;
const unsigned long GRID_OBSTACLE_NODE_REPEAT_MS = 900;
const unsigned long GRID_OBSTACLE_NODE_ARM_DELAY_MS = 250;
const unsigned long GRID_OBSTACLE_COOLDOWN_MS = 5000;
const float GRID_OBSTACLE_LINE_KP = 0.22;
const int16_t GRID_OBSTACLE_LINE_MAX_CORRECTION = 320;
const bool USE_WALL_FOLLOWING = true;
const bool WALL_FOLLOW_TEST_MODE = false; // Set true only when placing the robot directly in the tunnel.
const bool USE_FRONT_OBSTACLE_AVOIDANCE = true;
const float WALL_TARGET_CM = 15.0;
const float WALL_TOLERANCE_CM = 3.0;

const unsigned int SENSOR_TIMEOUT_US = 3000;
// Line acquisition is deliberately more permissive than corner detection.
// The lowered reflectance board reports the black line near 250-400us, while
// the bare floor is typically far lower. Keep turn detection strict below.
const unsigned int LINE_THRESHOLD = 220;
const unsigned int SIDE_CORRECTION_THRESHOLD = 430;
const unsigned int TURN_CORNER_SIDE_THRESHOLD = 300;       
const unsigned int T_JUNCTION_SIDE_THRESHOLD = 240;  // T needs both side pairs, so this can be lower than corner.
const unsigned int TURN_CORNER_CENTER_THRESHOLD = 280;
const int TURN_CORNER_SCORE_MIN = 650;
const int T_JUNCTION_SCORE_MIN = 460;
const bool ENABLE_T_JUNCTION_LATCH = true;
const unsigned int T_JUNCTION_LATCH_SIDE_THRESHOLD = 220;
const int T_JUNCTION_LATCH_SCORE_MIN = 430;
const uint16_t T_JUNCTION_LATCH_WINDOW_MS = 240;
const bool T_JUNCTION_DEBUG = true;
const unsigned long T_JUNCTION_DEBUG_INTERVAL_MS = 80;
const int TURN_CORNER_SCORE_MARGIN = 450;
const bool ENABLE_EARLY_LEFT_CORNER = false;
const unsigned int EARLY_LEFT_CORNER_SLIGHT_THRESHOLD = 400;
const int EARLY_LEFT_CORNER_RIGHT_SCORE_MAX = 60;
const unsigned int STRONG_SIDE_CORRECTION_THRESHOLD = 1100;
const bool BLACK_LINE_IS_HIGH = true;

const bool SENSOR_DEBUG_MODE = false;
const bool ENABLE_LINE_CORRECTION = true;
const bool USE_LINE_PID = true;
const bool ENABLE_RIGHT_ANGLE_TURNS = true;
const uint8_t BASE_UPHILL_ROUTE_TURN_COUNT = 3;
const uint8_t AIRLOCK_UPHILL_ROUTE_TURNS = 2; // After first RFID/open-airlock: left turn, then right branch turn.
const uint16_t AIRLOCK_UPHILL_LINE_ALIGN_MS = 1100;
const bool USE_IMU = true;
const bool USE_RFID = true;
const bool RFID_VERBOSE_DEBUG = false;

const int16_t MAX_SPEED = 1100;
// Conservative speed pass after the motor swap: line tracking is slowed more
// than turning so the new motor response can be re-calibrated safely.
const int16_t FORWARD_SPEED = 210;
const int16_t POST_TURN_FINE_FORWARD_SPEED = 250;
const int16_t POST_TURN_FINE_CORRECTION_SPEED = 520;
const int16_t MIN_CORRECTION_SPEED = 520;
const int16_t PIVOT_SPEED = 560;
const int16_t RIGHT_ANGLE_FAST_SPEED = 560;
const int16_t RIGHT_ANGLE_MID_SPEED = 500;
const int16_t RIGHT_ANGLE_FINE_SPEED = 340;
const int16_t RIGHT_ANGLE_RELIEF_SPEED = 420;
const int16_t FIELD_FORWARD_SPEED = 180;
const int16_t FIELD_POST_TURN_FINE_FORWARD_SPEED = 200;
const int16_t FIELD_POST_TURN_VERIFY_SPEED = 300;
const int16_t FIELD_FIND_LINE_SPEED = 360;
const int16_t FIELD_NO_LINE_FORWARD_SPEED = 220;
const int16_t FIELD_NO_LINE_YAW_MAX_CORRECTION = 60;
const int16_t FIELD_LINE_PID_MAX_CORRECTION = 180;
const uint8_t FIELD_TURN_SPEED_PERCENT = 80;
const bool ENABLE_ENCODER_TURN_BALANCE = true;
const uint16_t ENCODER_TURN_BALANCE_INTERVAL_MS = 120;
const uint8_t ENCODER_TURN_BALANCE_MIN_MOVING_WHEELS = 3;
const long ENCODER_TURN_BALANCE_MIN_TOTAL_TICKS = 4;
const float ENCODER_TURN_BALANCE_KP = 10.0;
const float ENCODER_TURN_BALANCE_RESPONSE = 0.35;
const int16_t ENCODER_TURN_BALANCE_MAX_TRIM = 120;
const bool ENABLE_ENCODER_SPEED_GOVERNOR = true;
const bool ENCODER_SPEED_GOVERNOR_DEBUG = true;
const uint16_t ENCODER_SPEED_GOVERNOR_INTERVAL_MS = 250;
const int16_t ENCODER_SPEED_GOVERNOR_COMMAND_CAP = 560;
const int16_t ENCODER_SPEED_GOVERNOR_MIN_ACTIVE_COMMAND = 160;
const int16_t ENCODER_SPEED_GOVERNOR_MAX_BRAKE = 80;
const int16_t ENCODER_SPEED_GOVERNOR_MAX_BOOST = 0;
const float ENCODER_SPEED_GOVERNOR_TARGET_TICKS_AT_CAP = 8.0;
const uint16_t ENCODER_SPEED_GOVERNOR_NOMINAL_INTERVAL_MS = 100;
const float ENCODER_SPEED_GOVERNOR_KP = 6.0;
const float ENCODER_SPEED_GOVERNOR_RESPONSE = 0.25;

// Pivot motor balance tuning. Fixed trims are now neutral; encoder feedback
// applies a side-level correction during IMU right-angle turns.
const uint8_t RIGHT_PIVOT_RF_PERCENT = 100;
const uint8_t RIGHT_PIVOT_RR_PERCENT = 100;
const uint8_t RIGHT_PIVOT_LR_PERCENT = 100;
const uint8_t RIGHT_PIVOT_LF_PERCENT = 100;

const uint8_t LEFT_PIVOT_RF_PERCENT = 100;
const uint8_t LEFT_PIVOT_RR_PERCENT = 100;
const uint8_t LEFT_PIVOT_LR_PERCENT = 100;
const uint8_t LEFT_PIVOT_LF_PERCENT = 100;
const int16_t NO_LINE_FORWARD_SPEED = 260;
const int16_t NO_LINE_YAW_MAX_CORRECTION = 120;
const float NO_LINE_YAW_KP = 18.0;
const unsigned int LINE_PID_SIGNAL_FLOOR = 180;
const int LINE_PID_MIN_SIGNAL_TOTAL = 20;
const int16_t LINE_PID_MAX_CORRECTION = 360;
const int16_t POST_TURN_LINE_PID_MAX_CORRECTION = 260;
const float LINE_PID_KP = 0.22;
const float LINE_PID_KI = 0.03;
const float LINE_PID_KD = 0.006;
const float LINE_PID_INTEGRAL_LIMIT = 3000.0;

const uint16_t FORWARD_PULSE_MS = 55;
const uint16_t POST_TURN_FINE_FORWARD_PULSE_MS = 35;
const uint16_t SENSOR_SETTLE_MS = 35;
const uint16_t PRE_CORRECT_STOP_MS = 10;
const uint16_t CORRECTION_PULSE_MS = 35;
const uint16_t STRONG_CORRECTION_PULSE_MS = 55;
const uint16_t POST_TURN_FINE_CORRECTION_PULSE_MS = 35;
const uint16_t AFTER_CORRECT_SETTLE_MS = 20;
const float PRE_TURN_FORWARD_LEFT_CM = 0.6;
const float PRE_TURN_FORWARD_LEFT_LATE_ROUTE_CM = 1.0;
const float PRE_TURN_FORWARD_RIGHT_CM = 0.35;
const float PRE_TURN_MS_PER_CM_AT_SPEED_700 = 55.6;
const float PRE_TURN_MS_PER_CM = PRE_TURN_MS_PER_CM_AT_SPEED_700 * 700.0 / FORWARD_SPEED;
const uint16_t PRE_TURN_EXTRA_MS = 120;
const uint16_t PRE_TURN_FORWARD_RIGHT_MS =
    (uint16_t)(PRE_TURN_FORWARD_RIGHT_CM * PRE_TURN_MS_PER_CM + 0.5) + PRE_TURN_EXTRA_MS;
const uint16_t AFTER_TURN_SETTLE_MS = 120;
const uint16_t RIGHT_ANGLE_TIMEOUT_MS = 14000;
const float RIGHT_ANGLE_TARGET_DEG = 90.0;
const float UTURN_TARGET_DEG = 180.0;
const float RIGHT_ANGLE_TOLERANCE_DEG = 1.5;
const bool ENABLE_TURN_LINE_ABORT = false;
const float TURN_LINE_ABORT_MIN_DEG = 45.0;
const float TURN_LINE_ABORT_EXTRA_DEG = 6.0;
const float RIGHT_ANGLE_MID_REMAINING_DEG = 28.0;
const float RIGHT_ANGLE_FINE_REMAINING_DEG = 9.0;
const float RIGHT_ANGLE_SEGMENT_DEG = 12.0;
const uint16_t RIGHT_ANGLE_SEGMENT_STOP_MS = 70;
const uint16_t RIGHT_ANGLE_RELIEF_PULSE_MS = 80;
const uint16_t RIGHT_ANGLE_SETTLE_MS = 180;
const uint16_t UTURN_TIMEOUT_MS = 26000;
const int16_t POST_TURN_VERIFY_SPEED = 360;
const uint16_t POST_TURN_VERIFY_MS = 420;
const uint16_t POST_TURN_FINE_LINE_MS = 1500;
const unsigned long POST_TURN_IGNORE_RIGHT_ANGLE_MS = 80;
const float POST_TURN_FRONT_BLOCK_CM = 7.0;
const float POST_TURN_MIN_TILT_DELTA_DEG = 4.0;
const long POST_TURN_MIN_ENCODER_TICKS = 3;
const int16_t POST_TURN_BACKUP_SPEED = 620;
const uint16_t POST_TURN_BACKUP_MS = 320;
const int16_t FIND_LINE_SPEED = 440;
const int16_t WALL_FORWARD_SPEED = 496;
const int16_t WALL_MIN_DRIVE_SPEED = 416;
const int16_t WALL_FIELD_APPROACH_SPEED = 380;
const int16_t WALL_FIELD_APPROACH_MIN_SPEED = 300;
const int16_t WALL_UPHILL_DOOR_HOLD_SPEED = 220;
const float WALL_SPEED_DECEL_PER_SEC = 150.0;
const float WALL_SPEED_ACCEL_PER_SEC = 260.0;
const uint16_t WALL_CLIMB_POWER_MS = 3000;
const uint16_t WALL_LINE_REACQUIRE_MIN_MS = 1800;
const uint16_t WALL_LINE_REACQUIRE_STABLE_MS = 250;
const uint16_t RETURN_BASE_REACQUIRE_SETTLE_MS = 500;
const uint16_t RETURN_BASE_IGNORE_TURN_MS = 1600;
const uint16_t RETURN_BASE_FINE_LINE_MS = 2500;
const float WALL_MIN_SIDE_CLEARANCE_CM = 4.5;
const float WALL_SIDE_CRITICAL_CM = 3.5;
const float WALL_SIDE_VALID_MAX_CM = 60.0;
const float WALL_CENTER_VALID_MAX_CM = 45.0;
const float WALL_FRONT_SLOW_CM = 45.0;
const float WALL_FRONT_STOP_CM = 15.0;
const int16_t WALL_PD_MAX_CORRECTION = 320;
const float WALL_CENTER_KP = 34.0;
const float WALL_CLEARANCE_KP = 165.0;
const float WALL_PD_KD = 4.0;
const float WALL_YAW_HOLD_KP = 12.0;
const float NO_LINE_WALL_ENTRY_SIDE_CM = 14.0;
const uint16_t NO_LINE_WALL_ENTRY_MIN_MS = 300;
const uint16_t NO_LINE_RETURN_YAW_TIMEOUT_MS = 1800;
const float NO_LINE_RETURN_YAW_TOLERANCE_DEG = 3.0;
const int16_t NO_LINE_RETURN_YAW_SPEED = 440;
const bool ENABLE_UPHILL_LINE_FLICKER_GUARD = true;
const uint16_t UPHILL_LINE_FLICKER_WINDOW_MS = 4500;
const uint8_t UPHILL_LINE_FLICKER_TRIGGER_TRANSITIONS = 2;
const uint16_t UPHILL_LINE_IGNORE_MS = 7000;
const bool ENABLE_UPHILL_TILT_GUARD = true;
const float UPHILL_TILT_TRIGGER_DEG = 4.0;
const float UPHILL_ROLL_TRIGGER_DEG = 3.0;
const float UPHILL_PITCH_TRIGGER_DEG = 3.5;
const uint16_t UPHILL_TILT_GUARD_MIN_MS = 180;
const uint16_t UPHILL_TILT_GUARD_COOLDOWN_MS = 2500;
const uint16_t FIND_LINE_SMALL_MS = 900;
const uint16_t FIND_LINE_BIG_MS = 3000;
const float FIND_LINE_SMALL_SWEEP_DEG = 24.0;
const float FIND_LINE_BIG_SWEEP_DEG = 70.0;
const unsigned long DEBOUNCE_MS = 50;
const unsigned long KILL_BLINK_MS = 250;
const unsigned long STARTUP_WIFI_SLOW_BLINK_MS = 800;
const unsigned long STARTUP_WIFI_FAST_BLINK_MS = 150;
const unsigned long EMERGENCY_LED_BLINK_MS = 180;
const unsigned long RFID_RETRY_INTERVAL_MS = 1000;
const unsigned long RFID_CHECK_INTERVAL_MS = 80;
const unsigned long RFID_DEBUG_INTERVAL_MS = 1000;
const uint32_t ULTRASONIC_TIMEOUT_US = 25000;
const unsigned long ULTRASONIC_SENSOR_GAP_MS = 30;
const unsigned long SENSOR_STATUS_INTERVAL_MS = 500;
const unsigned long SERIAL_PRINT_INTERVAL_MS = 100;
const unsigned long WALL_CONTROL_DEBUG_INTERVAL_MS = 250;

struct UltrasonicSensor
{
  const char *name;
  uint8_t trigPin;
  uint8_t echoPin;
  float distanceCm;
};

struct DebouncedButton
{
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

enum ServerRfidRequest
{
  SERVER_RFID_NONE,
  SERVER_RFID_WAIT_AIRLOCK,
  SERVER_RFID_WAIT_FERTILITY
};

unsigned int sensorValues[SENSOR_COUNT];
bool sensorOnLine[SENSOR_COUNT];
long encoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
long postTurnStartEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
bool lastEncoderA[ENCODER_COUNT] = {HIGH, HIGH, HIGH, HIGH};
long turnBalanceLastEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
long turnBalanceDeltaAbs[ENCODER_COUNT] = {0, 0, 0, 0};
int16_t turnBalanceTrim[ENCODER_COUNT] = {0, 0, 0, 0};
int16_t turnBalanceAppliedTrim[ENCODER_COUNT] = {0, 0, 0, 0};
bool turnBalanceEncoderActive[ENCODER_COUNT] = {false, false, false, false};
unsigned long turnBalanceLastMs = 0;
bool turnBalancePrimed = false;
bool turnBalanceHasSample = false;
int turnBalanceLastDirection = 0;
int16_t turnBalanceLastSpeed = 0;
long speedGovernorLastEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
float speedGovernorTrim[ENCODER_COUNT] = {0.0, 0.0, 0.0, 0.0};
long speedGovernorLastDelta[ENCODER_COUNT] = {0, 0, 0, 0};
float speedGovernorLastTarget[ENCODER_COUNT] = {0.0, 0.0, 0.0, 0.0};
int8_t speedGovernorLastSign[ENCODER_COUNT] = {0, 0, 0, 0};
int16_t speedGovernorLastRequested[ENCODER_COUNT] = {0, 0, 0, 0};
int16_t speedGovernorLastGoverned[ENCODER_COUNT] = {0, 0, 0, 0};
unsigned long speedGovernorLastMs = 0;
unsigned long speedGovernorLastDebugMs = 0;
bool speedGovernorPrimed = false;

int lastDirection = 0; // -1 = left, 0 = center/unknown, 1 = right
int correctionDirection = 0;
uint16_t activeCorrectionPulseMs = CORRECTION_PULSE_MS;
bool lastTurnDecisionWasTJunction = false;
unsigned long tJunctionRightSeenMs = 0;
unsigned long tJunctionLeftSeenMs = 0;
unsigned long lastTJunctionDebugMs = 0;
float linePidIntegral = 0.0;
float linePidLastError = 0.0;
unsigned long linePidLastMs = 0;
bool linePidHasLast = false;
unsigned long stateStartMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastStatePrintMs = 0;
unsigned long killPauseStartMs = 0;
unsigned long lastBlinkMs = 0;
unsigned long lastRFIDRetryMs = 0;
unsigned long lastRFIDCheckMs = 0;
unsigned long lastRFIDDebugMs = 0;
unsigned long lastRFIDActionMs = 0;
unsigned long lastUltrasonicReadMs = 0;
unsigned long lastSensorStatusMs = 0;
unsigned long ignoreRightAngleUntilMs = 0;
unsigned long postTurnFineLineUntilMs = 0;
unsigned long frontObstacleHoldStartMs = 0;
unsigned long ignoreFrontObstacleUntilMs = 0;
uint8_t nextUltrasonicIndex = 0;
float noLineTargetYawDeg = 0.0;
bool noLineYawTargetLocked = false;
unsigned long uphillLineFlickerWindowStartMs = 0;
unsigned long uphillLineIgnoreUntilMs = 0;
uint8_t uphillLineFlickerTransitions = 0;
bool uphillLineFlickerHasLast = false;
bool uphillLineFlickerLastSeen = false;
float uphillLineFlickerYawDeg = 0.0;
unsigned long uphillTiltGuardStartMs = 0;
unsigned long uphillTiltGuardLastTriggerMs = 0;
float uphillTiltGuardYawDeg = 0.0;
float wallPdLastError = 0.0;
unsigned long wallPdLastMs = 0;
bool wallPdHasLast = false;
float wallAppliedBaseSpeed = WALL_FORWARD_SPEED;
unsigned long wallSpeedRampLastMs = 0;
long wallFollowLastEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};
unsigned long wallFollowLastDebugMs = 0;
unsigned long wallLineReacquireSinceMs = 0;
int lastTurnDirection = 0; // -1 = left, 1 = right
uint8_t completedRouteTurns = 0;
int pendingTurnDirection = 0; // -1 = left, 1 = right
int findLineFirstDirection = 0; // -1 = left first, 1 = right first
float findLineStartYawDeg = 0.0;
int activeTurnDirection = 0; // -1 = left, 1 = right
int fineTurnDirection = 0; // -1 = left, 1 = right
float rightAngleSegmentTargetDeg = 0.0;
float activeTurnTargetDeg = RIGHT_ANGLE_TARGET_DEG;
bool turnLineAbortExtraActive = false;
float postTurnStartTiltDeg = 0.0;
float postTurnMaxTiltDeltaDeg = 0.0;
unsigned long rightAnglePhaseStartMs = 0;
bool nextTurnReliefForward = true;
int planterAngle = PLANTER_MIN_ANGLE;
unsigned long planterLastStepMs = 0;
unsigned long planterPauseStartMs = 0;
unsigned long lastPlantTriggerMs = 0;
bool runRandomActionAfterPlant = false;
bool killStopped = false;
bool startupWaitActive = true;
bool isRevived = false;
bool redBlinkEnabled = false;
bool blinkRedOn = false;
bool rfidReady = false;
bool rfidFirstCardSeen = false;
bool rfidPauseShouldPlant = false;
bool airlockRouteTracking = false;
uint8_t airlockRouteTurns = 0;
bool airlockDirectTunnelEntryArmed = false;
unsigned long airlockDirectTunnelEntryStartMs = 0;
uint32_t lastRFIDUidHash = 0;
bool haveLastRFIDUid = false;
unsigned long lastRFIDUidMs = 0;
uint32_t lastRFIDActionUidHash = 0;
bool haveLastRFIDActionUid = false;
char lastRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char activeRFIDTagId[RFID_TAG_ID_MAX_LEN] = "";
char plantingSeedTagId[RFID_TAG_ID_MAX_LEN] = "";
char pendingSeedReportTagId[RFID_TAG_ID_MAX_LEN] = "";
bool plantingSeedReportArmed = false;
bool pendingSeedReport = false;
bool serverMessengerStarted = false;
bool serverLastConnected = false;
bool serverHeartbeatAllowsMovement = true;
bool serverEmergencyActive = false;
bool serverDisableActive = false;
unsigned long serverLastRegisterMs = 0;
unsigned long serverLastStatusMs = 0;
ServerRfidRequest serverRfidRequest = SERVER_RFID_NONE;
unsigned long serverRfidRequestMs = 0;
bool serverRfidReplyReceived = false;
bool serverFertile = false;
bool serverPlanted = false;
bool serverAirlockAccepted = false;
bool serverShouldReportCurrentPlant = false;
bool returnModeActive = false;
bool returnPlanPending = false;
bool returnPositionKnown = false;
bool returnTunnelActive = false;
bool returnTunnelSawNoLine = false;
bool returnBaseReacquireSettleActive = false;
bool fieldModeActive = false;
bool fixedRecoveryActive = false;
int fixedRecoveryTurnDirection = 0;
int recoveryDirectionFallback = 1;
int8_t returnGridX = -1;
int8_t returnGridY = -1;
int8_t gridHeading = 0; // 0=N, 1=E, 2=S, 3=W.
uint32_t gridObstacleLastNodeUid = 0;
bool gridObstacleHasLastNode = false;
uint8_t gridObstacleNodeCount = 0;
uint8_t gridObstacleNodeTarget = 0;
int gridObstacleSide = 1; // -1 = left side bypass, 1 = right side bypass
int gridObstacleTurnDirection = 0;
float gridObstacleTurnStartYawDeg = 0.0;
unsigned long gridObstaclePhaseStartMs = 0;
unsigned long gridObstacleLastNodeMs = 0;
unsigned long gridObstacleTurnSettleStartMs = 0;
unsigned long gridObstacleLastCompleteMs = 0;
bool frontObstacleHolding = false;
DebouncedButton reviveButton = {REVIVE_BUTTON_PIN, HIGH, HIGH, 0};
DebouncedButton killButton = {KILL_BUTTON_PIN, HIGH, HIGH, 0};
Servo planterServo;

UltrasonicSensor ultrasonicSensors[] = {
    {"Front", US_FRONT_TRIG, US_FRONT_ECHO, -1.0},
    {"Left", US_LEFT_TRIG, US_LEFT_ECHO, -1.0},
    {"Right", US_RIGHT_TRIG, US_RIGHT_ECHO, -1.0},
};

const uint8_t NUM_ULTRASONIC = sizeof(ultrasonicSensors) / sizeof(ultrasonicSensors[0]);

const int16_t LF_TEST_SPEED = 500;
const uint16_t LF_TEST_RUN_MS = 1600;
const uint16_t LF_TEST_STOP_MS = 700;
const uint16_t LF_TEST_PRINT_MS = 100;

enum LfTestPhase
{
  LF_TEST_STOP_BEFORE,
  LF_TEST_FORWARD,
  LF_TEST_STOP_MID,
  LF_TEST_REVERSE,
  LF_TEST_STOP_AFTER
};

LfTestPhase lfTestPhase = LF_TEST_STOP_BEFORE;
unsigned long lfTestPhaseStartMs = 0;
unsigned long lfTestLastPrintMs = 0;
long lfTestLastEncoderCounts[ENCODER_COUNT] = {0, 0, 0, 0};

enum RightAnglePhase
{
  RIGHT_ANGLE_APPROACH,
  RIGHT_ANGLE_SEGMENT_STOP,
  RIGHT_ANGLE_RELIEF_FORWARD,
  RIGHT_ANGLE_RELIEF_BACKWARD,
  RIGHT_ANGLE_SETTLE,
  RIGHT_ANGLE_FINE_CORRECT
};

RightAnglePhase rightAnglePhase = RIGHT_ANGLE_APPROACH;

enum GridObstaclePhase
{
  GRID_OBS_IDLE,
  GRID_OBS_TURN_OUT,
  GRID_OBS_SIDE_STEP_OUT,
  GRID_OBS_TURN_FORWARD,
  GRID_OBS_PASS_OBSTACLE,
  GRID_OBS_TURN_BACK,
  GRID_OBS_SIDE_STEP_BACK,
  GRID_OBS_TURN_ORIGINAL,
  GRID_OBS_FORWARD_CLEAR,
  GRID_OBS_DONE
};

enum RobotState
{
  SENSE_LINE,
  FORWARD_PULSE,
  SENSOR_SETTLE,
  PRE_CORRECT_STOP,
  CORRECT_LEFT,
  CORRECT_RIGHT,
  AFTER_CORRECT_SETTLE,
  PRE_TURN_FORWARD_LEFT,
  PRE_TURN_FORWARD_RIGHT,
  TURN_LEFT_90,
  TURN_RIGHT_90,
  TURN_AROUND_180,
  POST_TURN_FORWARD_VERIFY,
  POST_TURN_BACK_UP,
  AFTER_TURN_SETTLE,
  FIND_LINE_SMALL_LEFT,
  FIND_LINE_SMALL_RIGHT,
  FIND_LINE_BIG_LEFT,
  FIND_LINE_BIG_RIGHT,
  NO_LINE_RETURN_YAW,
  NO_LINE_IMU_FORWARD,
  WALL_FOLLOW,
  GRID_OBSTACLE_AVOID,
  RFID_PAUSE,
  LINE_LOST_STOP
};

enum PlanterState
{
  PLANTER_IDLE,
  PLANTER_SWEEP_FORWARD,
  PLANTER_FORWARD_PAUSE,
  PLANTER_SWEEP_BACKWARD,
  PLANTER_BACKWARD_PAUSE
};

RobotState state = SENSE_LINE;
RobotState rfidReturnState = SENSE_LINE;
PlanterState planterState = PLANTER_IDLE;
GridObstaclePhase gridObstaclePhase = GRID_OBS_IDLE;

void enterState(RobotState nextState);
void enterNoLineFindState();
void printSensors(const char *label);
void printSensorsNow(const char *label);
const char *planterStateName();
bool triggerSeedPlanter(const char *source);
void handleRightRightAngleDecision();
void handleLeftRightAngleDecision();
void resetLinePid();
void startPostPlantRandomAction();
void setupServerMessenger();
void updateServerMessenger();
void onServerMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length);
bool serverStopActive();
void handleRFIDPauseState();
void linePidFollowStep();
void beginRightAngleTurn(int direction, float targetDeg = RIGHT_ANGLE_TARGET_DEG);
void beginTurnAround180();
void startGridObstacleAvoidance();
void gridObstacleAvoidStep();
float tiltMagnitudeDeg();
float calibratedPitchDeltaDeg();
float calibratedRollDeltaDeg();
float calibratedTiltMagnitudeDeg();
bool uphillLineIgnoreActive();
bool recordUphillLineFlicker(bool lineSeen, const char *source);
bool recordUphillTiltTransition(const char *source);
bool uphillTunnelPhaseReached();
bool wallFollowingAllowed();
bool sideWallCloseForNoLineEntry();
void enterUphillFlickerBypass(const char *source);
void startAirlockUphillRouteTracking(const char *source);
void recordAirlockRouteTurn(int direction);
bool airlockDirectTunnelEntryDue();
void enterAirlockDirectTunnelMode(const char *source);
void planEmergencyReturn();
void startFixedRecovery(const char *reason, int requestedDirection = 0);
bool fieldNavigationActive();
int returnCornerTurnDirectionRequested();

uint16_t preTurnMsForCm(float distanceCm)
{
  return (uint16_t)(distanceCm * PRE_TURN_MS_PER_CM + 0.5) + PRE_TURN_EXTRA_MS;
}

float activePreTurnForwardLeftCm()
{
  return completedRouteTurns >= 2
             ? PRE_TURN_FORWARD_LEFT_LATE_ROUTE_CM
             : PRE_TURN_FORWARD_LEFT_CM;
}

uint16_t activePreTurnForwardLeftMs()
{
  return preTurnMsForCm(activePreTurnForwardLeftCm());
}

const char *stateName(RobotState robotState)
{
  switch (robotState)
  {
    case SENSE_LINE: return "SENSE_LINE";
    case FORWARD_PULSE: return "FORWARD_PULSE";
    case SENSOR_SETTLE: return "SENSOR_SETTLE";
    case PRE_CORRECT_STOP: return "PRE_CORRECT_STOP";
    case CORRECT_LEFT: return "CORRECT_LEFT";
    case CORRECT_RIGHT: return "CORRECT_RIGHT";
    case AFTER_CORRECT_SETTLE: return "AFTER_CORRECT_SETTLE";
    case PRE_TURN_FORWARD_LEFT: return "PRE_TURN_FORWARD_LEFT";
    case PRE_TURN_FORWARD_RIGHT: return "PRE_TURN_FORWARD_RIGHT";
    case TURN_LEFT_90: return "TURN_LEFT_90";
    case TURN_RIGHT_90: return "TURN_RIGHT_90";
    case TURN_AROUND_180: return "TURN_AROUND_180";
    case POST_TURN_FORWARD_VERIFY: return "POST_TURN_FORWARD_VERIFY";
    case POST_TURN_BACK_UP: return "POST_TURN_BACK_UP";
    case AFTER_TURN_SETTLE: return "AFTER_TURN_SETTLE";
    case FIND_LINE_SMALL_LEFT: return "FIND_LINE_SMALL_LEFT";
    case FIND_LINE_SMALL_RIGHT: return "FIND_LINE_SMALL_RIGHT";
    case FIND_LINE_BIG_LEFT: return "FIND_LINE_BIG_LEFT";
    case FIND_LINE_BIG_RIGHT: return "FIND_LINE_BIG_RIGHT";
    case NO_LINE_RETURN_YAW: return "NO_LINE_RETURN_YAW";
    case NO_LINE_IMU_FORWARD: return "NO_LINE_IMU_FORWARD";
    case WALL_FOLLOW: return "WALL_FOLLOW";
    case GRID_OBSTACLE_AVOID: return "GRID_OBSTACLE_AVOID";
    case RFID_PAUSE: return "RFID_PAUSE";
    case LINE_LOST_STOP: return "LINE_LOST_STOP";
  }
  return "UNKNOWN";
}

int16_t applyEncoderSpeedGovernorToWheel(uint8_t index, int16_t requested)
{
  if (!ENABLE_ENCODER_SPEED_GOVERNOR)
  {
    return constrain(requested, -MAX_SPEED, MAX_SPEED);
  }

  int16_t requestedMag = abs(requested);
  int8_t requestedSign = requested > 0 ? 1 : (requested < 0 ? -1 : 0);
  if (requestedMag < ENCODER_SPEED_GOVERNOR_MIN_ACTIVE_COMMAND || requestedSign == 0)
  {
    speedGovernorTrim[index] = 0.0;
    speedGovernorLastTarget[index] = 0.0;
    speedGovernorLastSign[index] = requestedSign;
    return 0;
  }

  if (speedGovernorLastSign[index] != requestedSign)
  {
    speedGovernorTrim[index] = 0.0;
    speedGovernorLastSign[index] = requestedSign;
  }

  int16_t cappedMag = min(requestedMag, ENCODER_SPEED_GOVERNOR_COMMAND_CAP);
  int16_t governedMag = constrain(
      (int16_t)(cappedMag + speedGovernorTrim[index]),
      (int16_t)0,
      (int16_t)MAX_SPEED);

  return requestedSign * governedMag;
}

void updateEncoderSpeedGovernor()
{
  if (!ENABLE_ENCODER_SPEED_GOVERNOR)
  {
    return;
  }

  unsigned long now = millis();
  if (!speedGovernorPrimed)
  {
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      speedGovernorLastEncoderCounts[i] = encoderCounts[i];
    }
    speedGovernorLastMs = now;
    speedGovernorPrimed = true;
    return;
  }

  if (now - speedGovernorLastMs < ENCODER_SPEED_GOVERNOR_INTERVAL_MS)
  {
    return;
  }

  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    long delta = labs(encoderCounts[i] - speedGovernorLastEncoderCounts[i]);
    speedGovernorLastDelta[i] = delta;
    speedGovernorLastEncoderCounts[i] = encoderCounts[i];

    if (speedGovernorLastTarget[i] <= 0.0)
    {
      speedGovernorTrim[i] = 0.0;
      continue;
    }

    float error = speedGovernorLastTarget[i] - (float)delta;
    float desiredTrim = constrain(
        error * ENCODER_SPEED_GOVERNOR_KP,
        (float)-ENCODER_SPEED_GOVERNOR_MAX_BRAKE,
        (float) ENCODER_SPEED_GOVERNOR_MAX_BOOST);
    speedGovernorTrim[i] +=
        (desiredTrim - speedGovernorTrim[i]) * ENCODER_SPEED_GOVERNOR_RESPONSE;
    speedGovernorTrim[i] = constrain(
        speedGovernorTrim[i],
        (float)-ENCODER_SPEED_GOVERNOR_MAX_BRAKE,
        (float) ENCODER_SPEED_GOVERNOR_MAX_BOOST);
  }

  speedGovernorLastMs = now;

}

void printEncoderSpeedGovernorDebug()
{
  if (!ENCODER_SPEED_GOVERNOR_DEBUG)
  {
    return;
  }

  unsigned long now = millis();
  if (now - speedGovernorLastDebugMs >= 500)
  {
    speedGovernorLastDebugMs = now;
    Serial.print("SPEED_GOV | req=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(speedGovernorLastRequested[i]);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.print(" out=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(speedGovernorLastGoverned[i]);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.print(" target=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(speedGovernorLastTarget[i], 1);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.print(" delta=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(speedGovernorLastDelta[i]);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.print(" trim=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(speedGovernorTrim[i], 0);
      Serial.print(i + 1 < ENCODER_COUNT ? "," : "");
    }
    Serial.print(" active=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(abs(speedGovernorLastGoverned[i]) >= ENCODER_SPEED_GOVERNOR_MIN_ACTIVE_COMMAND ? "1" : "0");
    }
    Serial.println();
  }
}

void setMotorSpeeds(int16_t rf, int16_t rr, int16_t lr, int16_t lf)
{
  int16_t requested[ENCODER_COUNT] = {
      (int16_t)constrain(rf, -MAX_SPEED, MAX_SPEED),
      (int16_t)constrain(rr, -MAX_SPEED, MAX_SPEED),
      (int16_t)constrain(lr, -MAX_SPEED, MAX_SPEED),
      (int16_t)constrain(lf, -MAX_SPEED, MAX_SPEED),
  };

  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    speedGovernorLastRequested[i] = requested[i];

    int16_t requestedMag = abs(requested[i]);
    if (requestedMag >= ENCODER_SPEED_GOVERNOR_MIN_ACTIVE_COMMAND)
    {
      int16_t cappedMag = min(requestedMag, ENCODER_SPEED_GOVERNOR_COMMAND_CAP);
      speedGovernorLastTarget[i] =
          ENCODER_SPEED_GOVERNOR_TARGET_TICKS_AT_CAP *
          (float)cappedMag /
          (float)ENCODER_SPEED_GOVERNOR_COMMAND_CAP *
          (float)ENCODER_SPEED_GOVERNOR_INTERVAL_MS /
          (float)ENCODER_SPEED_GOVERNOR_NOMINAL_INTERVAL_MS;
    }
    else
    {
      speedGovernorLastTarget[i] = 0.0;
    }
  }

  updateEncoderSpeedGovernor();

  int16_t governed[ENCODER_COUNT] = {
      applyEncoderSpeedGovernorToWheel(0, requested[0]),
      applyEncoderSpeedGovernorToWheel(1, requested[1]),
      applyEncoderSpeedGovernorToWheel(2, requested[2]),
      applyEncoderSpeedGovernorToWheel(3, requested[3]),
  };

  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    speedGovernorLastGoverned[i] = governed[i];
  }
  printEncoderSpeedGovernorDebug();

  mcLower.setSpeed(RF, governed[0]);
  mcLower.setSpeed(RR, governed[1]);
  mcLower.setSpeed(LR, governed[2]);
  mcUpper.setSpeed(LF, governed[3]);
}

void stopAllMotors()
{
  setMotorSpeeds(0, 0, 0, 0);
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
    postTurnStartEncoderCounts[i] = 0;
  }
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

  if ((millis() - button.lastChangeMs) > DEBOUNCE_MS && reading != button.stableState)
  {
    button.stableState = reading;
    return button.stableState == LOW;
  }

  return false;
}

void updateLed()
{
  if (serverEmergencyActive)
  {
    if (millis() - lastBlinkMs >= EMERGENCY_LED_BLINK_MS)
    {
      lastBlinkMs = millis();
      blinkRedOn = !blinkRedOn;
    }
    digitalWrite(LED_RED_PIN, blinkRedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, blinkRedOn ? LOW : HIGH);
    return;
  }

  if (redBlinkEnabled)
  {
    const unsigned long blinkIntervalMs = startupWaitActive
                                          ? (serverLastConnected ? STARTUP_WIFI_FAST_BLINK_MS : STARTUP_WIFI_SLOW_BLINK_MS)
                                          : KILL_BLINK_MS;
    if (millis() - lastBlinkMs >= blinkIntervalMs)
    {
      lastBlinkMs = millis();
      blinkRedOn = !blinkRedOn;
    }
    digitalWrite(LED_RED_PIN, blinkRedOn ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }

  digitalWrite(LED_RED_PIN, isRevived ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, isRevived ? HIGH : LOW);
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
    isRevived = false;
    redBlinkEnabled = true;
    blinkRedOn = true;
    lastBlinkMs = millis();
    killPauseStartMs = millis();
    stopAllMotors();
  }
  else
  {
    startupWaitActive = false;
    redBlinkEnabled = false;
    if (killPauseStartMs != 0)
    {
      stateStartMs += millis() - killPauseStartMs;
      killPauseStartMs = 0;
    }
    lastUpdateMicros = micros();
  }
}

void setReviveLedGreen(const char *source)
{
  isRevived = true;
  redBlinkEnabled = false;

  Serial.print("REVIVE | ");
  Serial.print(source);
  Serial.println(" -> LED GREEN");
}

void updateButtons()
{
  if (digitalRead(REVIVE_BUTTON_PIN) == LOW)
  {
    setReviveLedGreen("D32 level");
  }

  if (buttonPressed(reviveButton))
  {
    setReviveLedGreen("D32");
  }

  if (buttonPressed(killButton))
  {
    setKillStopped(!killStopped, "Mechanical D45");
  }
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
  snprintf(payload,
           sizeof(payload),
           "type=register team_id=%s board_id=%s",
           GROUP_ID,
           BOARD_ID);
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

bool sendServerOpenAirlock(const char *tagId)
{
  char payload[160];
  snprintf(payload,
           sizeof(payload),
           "type=%s team_id=%s airlock=%s tag_id=%s board_id=%s",
           SERVER_AIRLOCK_OPEN_TYPE,
           GROUP_ID,
           SERVER_AIRLOCK_ID,
           tagId,
           BOARD_ID);
  return sendServerText(payload);
}

void setupServerMessenger()
{
  if (!USE_SERVER_MESSENGER)
  {
    Serial.println("SERVER | Offline mode. MiniMessenger disabled.");
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
  serverLastConnected = connected;

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
      Serial.println("SERVER | seedPlanted acknowledged locally after TX.");
    }
  }
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

  if (messageTypeEquals(message, "heartbeat"))
  {
    bool enabled = true;
    if (parseBoolField(message, "enable", enabled))
    {
      serverHeartbeatAllowsMovement = enabled;
      if (!enabled)
      {
        stopAllMotors();
        Serial.println("SERVER | Heartbeat disabled movement.");
      }
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
        blinkRedOn = true;
        lastBlinkMs = millis();
        stopAllMotors();
        returnModeActive = true;
        returnTunnelActive = false;
        returnTunnelSawNoLine = false;
        returnPlanPending = true;
        Serial.println("SERVER | Emergency active. Stop current action and return to (9,7).");
      }
      else
      {
        Serial.println("SERVER | Emergency cleared.");
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
        Serial.println("SERVER | Robot disabled by server.");
      }
      else
      {
        Serial.println("SERVER | Robot enabled by server.");
      }
    }
    return;
  }

  if (messageTypeEquals(message, "isFertileReply"))
  {
    bool fertile = false;
    bool planted = false;
    bool haveFertile = parseBoolField(message, "fertile", fertile);
    bool havePlanted = parseBoolField(message, "planted", planted);

    serverFertile = haveFertile && fertile;
    serverPlanted = havePlanted && planted;
    int x = -1;
    int y = -1;
    if (parseIntField(message, "x", x) && parseIntField(message, "y", y))
    {
      returnGridX = (int8_t)x;
      returnGridY = (int8_t)y;
      returnPositionKnown = true;
      Serial.print("LOCALIZE | x=");
      Serial.print(returnGridX);
      Serial.print(" y=");
      Serial.print(returnGridY);
      Serial.print(" heading=");
      Serial.println(gridHeading);
    }
    serverRfidReplyReceived = true;

    Serial.print("SERVER | Fertility reply fertile=");
    Serial.print(serverFertile ? "true" : "false");
    Serial.print(" planted=");
    Serial.println(serverPlanted ? "true" : "false");

    if (serverRfidRequest == SERVER_RFID_WAIT_FERTILITY)
    {
      rfidPauseShouldPlant = serverFertile && !serverPlanted;
      serverShouldReportCurrentPlant = rfidPauseShouldPlant;
    }
    return;
  }

  if (messageTypeEquals(message, "openAirlockReply"))
  {
    bool accepted = false;
    if (parseBoolField(message, "accepted", accepted))
    {
      serverAirlockAccepted = accepted;
    }
    serverRfidReplyReceived = true;

    Serial.print("SERVER | Airlock reply accepted=");
    Serial.println(serverAirlockAccepted ? "true" : "false");
    return;
  }
}

bool serverStopActive()
{
  return USE_SERVER_MESSENGER &&
         serverMessengerStarted &&
         (!serverHeartbeatAllowsMovement || serverDisableActive);
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

int16_t scaledSpeed(int16_t speed, uint8_t percent)
{
  long scaled = (long)speed * percent / 100;
  return (int16_t)scaled;
}

void pivotRight(int16_t speed)
{
  // Right pivot:
  // right side backward, left side forward.
  // Keep all motors at 100% because right turns are already stable.

  setMotorSpeeds(
      scaledSpeed(-speed, RIGHT_PIVOT_RF_PERCENT) * FWD_RF,
      scaledSpeed(-speed, RIGHT_PIVOT_RR_PERCENT) * FWD_RR,
      scaledSpeed( speed, RIGHT_PIVOT_LR_PERCENT) * FWD_LR,
      scaledSpeed( speed, RIGHT_PIVOT_LF_PERCENT) * FWD_LF);
}

void pivotLeft(int16_t speed)
{
  // Left pivot:
  // left side backward, right side forward.

  setMotorSpeeds(
      scaledSpeed( speed, LEFT_PIVOT_RF_PERCENT) * FWD_RF,
      scaledSpeed( speed, LEFT_PIVOT_RR_PERCENT) * FWD_RR,
      scaledSpeed(-speed, LEFT_PIVOT_LR_PERCENT) * FWD_LR,
      scaledSpeed(-speed, LEFT_PIVOT_LF_PERCENT) * FWD_LF);
}

void resetTurnEncoderBalance()
{
  copyEncoderCounts(turnBalanceLastEncoderCounts);
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    turnBalanceTrim[i] = 0;
    turnBalanceAppliedTrim[i] = 0;
    turnBalanceDeltaAbs[i] = 0;
    turnBalanceEncoderActive[i] = false;
  }
  turnBalanceLastMs = millis();
  turnBalancePrimed = true;
  turnBalanceHasSample = false;
  turnBalanceLastDirection = 0;
  turnBalanceLastSpeed = 0;
}

void updateTurnEncoderBalance(int direction, int16_t speed)
{
  if (!ENABLE_ENCODER_TURN_BALANCE)
  {
    return;
  }

  unsigned long now = millis();
  if (!turnBalancePrimed ||
      direction != turnBalanceLastDirection ||
      speed != turnBalanceLastSpeed)
  {
    copyEncoderCounts(turnBalanceLastEncoderCounts);
    turnBalanceLastMs = now;
    turnBalancePrimed = true;
    turnBalanceLastDirection = direction;
    turnBalanceLastSpeed = speed;
    return;
  }

  if (now - turnBalanceLastMs < ENCODER_TURN_BALANCE_INTERVAL_MS)
  {
    return;
  }

  long deltaAbs[ENCODER_COUNT];
  long totalTicks = 0;
  uint8_t activeWheels = 0;
  long rightTotalTicks = 0;
  long leftTotalTicks = 0;
  uint8_t rightActiveWheels = 0;
  uint8_t leftActiveWheels = 0;
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    deltaAbs[i] = labs(encoderCounts[i] - turnBalanceLastEncoderCounts[i]);
    turnBalanceDeltaAbs[i] = deltaAbs[i];
    turnBalanceLastEncoderCounts[i] = encoderCounts[i];
    totalTicks += deltaAbs[i];
    if (deltaAbs[i] > 0)
    {
      turnBalanceEncoderActive[i] = true;
    }

    if (turnBalanceEncoderActive[i])
    {
      activeWheels++;
      if (i <= 1)
      {
        rightTotalTicks += deltaAbs[i];
        rightActiveWheels++;
      }
      else
      {
        leftTotalTicks += deltaAbs[i];
        leftActiveWheels++;
      }
    }
  }

  turnBalanceLastMs = now;

  if (activeWheels < ENCODER_TURN_BALANCE_MIN_MOVING_WHEELS ||
      totalTicks < ENCODER_TURN_BALANCE_MIN_TOTAL_TICKS ||
      rightActiveWheels == 0 ||
      leftActiveWheels == 0)
  {
    turnBalanceHasSample = false;
    return;
  }

  turnBalanceHasSample = true;
  float rightAvgTicks = (float)rightTotalTicks / (float)rightActiveWheels;
  float leftAvgTicks = (float)leftTotalTicks / (float)leftActiveWheels;
  float sideErrorTicks = leftAvgTicks - rightAvgTicks;
  float desiredRightTrim = constrain(
      sideErrorTicks * ENCODER_TURN_BALANCE_KP,
      (float)-ENCODER_TURN_BALANCE_MAX_TRIM,
      (float) ENCODER_TURN_BALANCE_MAX_TRIM);
  float desiredLeftTrim = -desiredRightTrim;
  float currentRightTrim = ((float)turnBalanceTrim[0] + (float)turnBalanceTrim[1]) * 0.5;
  float currentLeftTrim = ((float)turnBalanceTrim[2] + (float)turnBalanceTrim[3]) * 0.5;
  float nextRightTrim = currentRightTrim +
                        (desiredRightTrim - currentRightTrim) *
                            ENCODER_TURN_BALANCE_RESPONSE;
  float nextLeftTrim = currentLeftTrim +
                       (desiredLeftTrim - currentLeftTrim) *
                           ENCODER_TURN_BALANCE_RESPONSE;

  int16_t rightTrim = constrain(
      (int16_t)(nextRightTrim + (nextRightTrim >= 0.0 ? 0.5 : -0.5)),
      (int16_t)-ENCODER_TURN_BALANCE_MAX_TRIM,
      (int16_t) ENCODER_TURN_BALANCE_MAX_TRIM);
  int16_t leftTrim = constrain(
      (int16_t)(nextLeftTrim + (nextLeftTrim >= 0.0 ? 0.5 : -0.5)),
      (int16_t)-ENCODER_TURN_BALANCE_MAX_TRIM,
      (int16_t) ENCODER_TURN_BALANCE_MAX_TRIM);

  turnBalanceTrim[0] = rightTrim;
  turnBalanceTrim[1] = rightTrim;
  turnBalanceTrim[2] = leftTrim;
  turnBalanceTrim[3] = leftTrim;
}

void updateAppliedTurnBalanceTrims()
{
  turnBalanceAppliedTrim[0] = turnBalanceTrim[0];
  turnBalanceAppliedTrim[1] = turnBalanceTrim[1];
  turnBalanceAppliedTrim[2] = turnBalanceTrim[2];
  turnBalanceAppliedTrim[3] = turnBalanceTrim[3];
}

int16_t turnWheelMagnitude(int16_t speed, uint8_t percent, int16_t trim)
{
  return constrain(
      (int16_t)(scaledSpeed(speed, percent) + trim),
      (int16_t)0,
      (int16_t)MAX_SPEED);
}

void balancedPivot(int direction, int16_t speed)
{
  updateTurnEncoderBalance(direction, speed);
  updateAppliedTurnBalanceTrims();

  uint8_t rfPercent = direction < 0 ? LEFT_PIVOT_RF_PERCENT : RIGHT_PIVOT_RF_PERCENT;
  uint8_t rrPercent = direction < 0 ? LEFT_PIVOT_RR_PERCENT : RIGHT_PIVOT_RR_PERCENT;
  uint8_t lrPercent = direction < 0 ? LEFT_PIVOT_LR_PERCENT : RIGHT_PIVOT_LR_PERCENT;
  uint8_t lfPercent = direction < 0 ? LEFT_PIVOT_LF_PERCENT : RIGHT_PIVOT_LF_PERCENT;

  int16_t rfMag = turnWheelMagnitude(speed, rfPercent, turnBalanceAppliedTrim[0]);
  int16_t rrMag = turnWheelMagnitude(speed, rrPercent, turnBalanceAppliedTrim[1]);
  int16_t lrMag = turnWheelMagnitude(speed, lrPercent, turnBalanceAppliedTrim[2]);
  int16_t lfMag = turnWheelMagnitude(speed, lfPercent, turnBalanceAppliedTrim[3]);

  int16_t rightSign = direction < 0 ? 1 : -1;
  int16_t leftSign = direction < 0 ? -1 : 1;
  setMotorSpeeds(
      rightSign * rfMag * FWD_RF,
      rightSign * rrMag * FWD_RR,
      leftSign * lrMag * FWD_LR,
      leftSign * lfMag * FWD_LF);
}
void scanI2CBus(TwoWire &bus, const char *name)
{
  Serial.print("Scanning ");
  Serial.print(name);
  Serial.println("...");

  for (uint8_t address = 1; address < 127; address++)
  {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();

    if (error == 0)
    {
      Serial.print("  Found I2C device at 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
}

bool initIMU()
{
  const bool ad0Values[] = {false, true};

  for (uint8_t i = 0; i < 2; i++)
  {
    imuAd0Val = ad0Values[i];

    Serial.print("IMU | Trying AD0=");
    Serial.print(imuAd0Val ? 1 : 0);
    Serial.print(" address 0x");
    Serial.println(imuAd0Val ? "69" : "68");

    imu.begin(Wire, imuAd0Val);
    Serial.print("IMU | Status: ");
    Serial.println(imu.statusString());

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
  Serial.println("IMU | Calibrating gyro Z. Keep the car still...");

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

  Serial.print("IMU | Gyro Z bias = ");
  Serial.print(gyroZBias, 6);
  Serial.print(" from ");
  Serial.print(count);
  Serial.println(" samples");
}

void updateYaw()
{
  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();

  float ax = imu.accX();
  float ay = imu.accY();
  float az = imu.accZ();
  float horizontal = sqrt(ax * ax + az * az);
  if (horizontal > 1.0 || fabs(ay) > 1.0)
  {
    pitchDeg = atan2(ay, horizontal) * 180.0 / PI;
    rollDeg = atan2(ax, az) * 180.0 / PI;
  }

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
  float error = currentDeg - referenceDeg;
  while (error > 180.0)
  {
    error -= 360.0;
  }
  while (error < -180.0)
  {
    error += 360.0;
  }
  return error;
}

void captureTiltBaseline()
{
  const uint8_t samples = 30;
  const unsigned long timeoutMs = 1200;
  float pitchSum = 0.0;
  float rollSinSum = 0.0;
  float rollCosSum = 0.0;
  uint8_t count = 0;
  unsigned long startMs = millis();

  while (count < samples && millis() - startMs < timeoutMs)
  {
    if (!imu.dataReady())
    {
      delay(5);
      continue;
    }

    updateYaw();
    float rollRad = rollDeg * PI / 180.0;
    pitchSum += pitchDeg;
    rollSinSum += sin(rollRad);
    rollCosSum += cos(rollRad);
    count++;
    delay(5);
  }

  if (count == 0)
  {
    tiltBaselineCaptured = false;
    Serial.println("IMU | Tilt baseline not captured.");
    return;
  }

  tiltBaselinePitchDeg = pitchSum / (float)count;
  tiltBaselineRollDeg = atan2(rollSinSum, rollCosSum) * 180.0 / PI;
  tiltBaselineCaptured = true;

  Serial.print("IMU | Tilt baseline pitch=");
  Serial.print(tiltBaselinePitchDeg, 1);
  Serial.print(" roll=");
  Serial.print(tiltBaselineRollDeg, 1);
  Serial.print(" samples=");
  Serial.println(count);
}

float rightAngleTurnDeg()
{
  return fabs(yawDeg - turnStartYawDeg);
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

bool middleBandOnLine()
{
  return sensorOnLine[SENSOR_SLIGHT_RIGHT] ||
         sensorOnLine[SENSOR_MIDDLE] ||
         sensorOnLine[SENSOR_SLIGHT_LEFT];
}

bool centerOrAnySensorOnLine()
{
  readSensors();
  printSensors("FIND_LINE");
  return anySensorOnLine();
}

bool postTurnFineLineActive()
{
  return millis() < postTurnFineLineUntilMs;
}

bool sensorAboveCorrectionThreshold(uint8_t index)
{
  return sensorValues[index] >= SIDE_CORRECTION_THRESHOLD;
}

bool leftCorrectionRequested()
{
  return sensorAboveCorrectionThreshold(SENSOR_LEFT);
}

bool rightCorrectionRequested()
{
  return sensorAboveCorrectionThreshold(SENSOR_RIGHT);
}

bool sensorAboveStrongCorrectionThreshold(uint8_t index)
{
  return sensorValues[index] >= STRONG_SIDE_CORRECTION_THRESHOLD;
}

bool leftStrongCorrectionRequested()
{
  return sensorAboveStrongCorrectionThreshold(SENSOR_LEFT);
}

bool rightStrongCorrectionRequested()
{
  return sensorAboveStrongCorrectionThreshold(SENSOR_RIGHT);
}

int rightAngleTurnDirectionRequested()
{
  lastTurnDecisionWasTJunction = false;
  unsigned long nowMs = millis();
  bool centerOnLine = sensorValues[SENSOR_MIDDLE] >= TURN_CORNER_CENTER_THRESHOLD;
  int rightScore = sensorValues[SENSOR_RIGHT] + sensorValues[SENSOR_SLIGHT_RIGHT];
  int leftScore = sensorValues[SENSOR_SLIGHT_LEFT] + sensorValues[SENSOR_LEFT];
  bool rightPairPresent = sensorValues[SENSOR_RIGHT] >= TURN_CORNER_SIDE_THRESHOLD &&
                          sensorValues[SENSOR_SLIGHT_RIGHT] >= TURN_CORNER_SIDE_THRESHOLD;
  bool leftPairPresent = sensorValues[SENSOR_SLIGHT_LEFT] >= TURN_CORNER_SIDE_THRESHOLD &&
                         sensorValues[SENSOR_LEFT] >= TURN_CORNER_SIDE_THRESHOLD;
  bool rightTJunctionPairPresent = sensorValues[SENSOR_RIGHT] >= T_JUNCTION_SIDE_THRESHOLD &&
                                   sensorValues[SENSOR_SLIGHT_RIGHT] >= T_JUNCTION_SIDE_THRESHOLD;
  bool leftTJunctionPairPresent = sensorValues[SENSOR_SLIGHT_LEFT] >= T_JUNCTION_SIDE_THRESHOLD &&
                                  sensorValues[SENSOR_LEFT] >= T_JUNCTION_SIDE_THRESHOLD;
  bool rightTJunctionLatchPresent = sensorValues[SENSOR_RIGHT] >= T_JUNCTION_LATCH_SIDE_THRESHOLD &&
                                    sensorValues[SENSOR_SLIGHT_RIGHT] >= T_JUNCTION_LATCH_SIDE_THRESHOLD &&
                                    rightScore >= T_JUNCTION_LATCH_SCORE_MIN;
  bool leftTJunctionLatchPresent = sensorValues[SENSOR_SLIGHT_LEFT] >= T_JUNCTION_LATCH_SIDE_THRESHOLD &&
                                   sensorValues[SENSOR_LEFT] >= T_JUNCTION_LATCH_SIDE_THRESHOLD &&
                                   leftScore >= T_JUNCTION_LATCH_SCORE_MIN;
  bool rightCorner = centerOnLine &&
                     rightPairPresent &&
                     rightScore >= TURN_CORNER_SCORE_MIN &&
                     rightScore - leftScore >= TURN_CORNER_SCORE_MARGIN;
  bool leftCorner = centerOnLine &&
                    leftPairPresent &&
                    leftScore >= TURN_CORNER_SCORE_MIN &&
                    leftScore - rightScore >= TURN_CORNER_SCORE_MARGIN;
  bool earlyLeftCorner = ENABLE_EARLY_LEFT_CORNER &&
                         completedRouteTurns > 0 &&
                         centerOnLine &&
                         sensorValues[SENSOR_SLIGHT_LEFT] >= EARLY_LEFT_CORNER_SLIGHT_THRESHOLD &&
                         rightScore <= EARLY_LEFT_CORNER_RIGHT_SCORE_MAX;

  if (rightTJunctionLatchPresent)
  {
    tJunctionRightSeenMs = nowMs;
  }

  if (leftTJunctionLatchPresent)
  {
    tJunctionLeftSeenMs = nowMs;
  }

  bool recentRightTJunction = tJunctionRightSeenMs != 0 &&
                              nowMs - tJunctionRightSeenMs <= T_JUNCTION_LATCH_WINDOW_MS;
  bool recentLeftTJunction = tJunctionLeftSeenMs != 0 &&
                             nowMs - tJunctionLeftSeenMs <= T_JUNCTION_LATCH_WINDOW_MS;
  bool latchedTJunction = ENABLE_T_JUNCTION_LATCH &&
                          recentRightTJunction &&
                          recentLeftTJunction;

  if (T_JUNCTION_DEBUG &&
      (rightTJunctionLatchPresent || leftTJunctionLatchPresent || latchedTJunction) &&
      nowMs - lastTJunctionDebugMs >= T_JUNCTION_DEBUG_INTERVAL_MS)
  {
    lastTJunctionDebugMs = nowMs;
    Serial.print("TJ_DEBUG | R=");
    Serial.print(rightScore);
    Serial.print(rightTJunctionPairPresent ? " pair" : " -");
    Serial.print(rightTJunctionLatchPresent ? " latch" : "");
    Serial.print(" L=");
    Serial.print(leftScore);
    Serial.print(leftTJunctionPairPresent ? " pair" : " -");
    Serial.print(leftTJunctionLatchPresent ? " latch" : "");
    Serial.print(" center=");
    Serial.print(sensorValues[SENSOR_MIDDLE]);
    Serial.print(" recentR=");
    Serial.print(recentRightTJunction ? (int)(nowMs - tJunctionRightSeenMs) : -1);
    Serial.print("ms recentL=");
    Serial.print(recentLeftTJunction ? (int)(nowMs - tJunctionLeftSeenMs) : -1);
    Serial.print("ms yaw=");
    Serial.println(yawDeg, 1);
  }

  if (rightTJunctionPairPresent &&
      leftTJunctionPairPresent &&
      rightScore >= T_JUNCTION_SCORE_MIN &&
      leftScore >= T_JUNCTION_SCORE_MIN)
  {
    tJunctionRightSeenMs = 0;
    tJunctionLeftSeenMs = 0;
    lastTurnDecisionWasTJunction = true;
    return 1;
  }

  if (latchedTJunction)
  {
    tJunctionRightSeenMs = 0;
    tJunctionLeftSeenMs = 0;
    lastTurnDecisionWasTJunction = true;
    return 1;
  }

  if (earlyLeftCorner)
  {
    return -1;
  }

  if (rightCorner && !leftCorner)
  {
    return 1;
  }

  if (leftCorner && !rightCorner)
  {
    return -1;
  }

  return 0;
}

int returnCornerTurnDirectionRequested()
{
  bool centerOnLine = sensorValues[SENSOR_MIDDLE] >= TURN_CORNER_CENTER_THRESHOLD;
  int rightScore = sensorValues[SENSOR_RIGHT] + sensorValues[SENSOR_SLIGHT_RIGHT];
  int leftScore = sensorValues[SENSOR_SLIGHT_LEFT] + sensorValues[SENSOR_LEFT];
  bool rightPairPresent = sensorValues[SENSOR_RIGHT] >= TURN_CORNER_SIDE_THRESHOLD &&
                          sensorValues[SENSOR_SLIGHT_RIGHT] >= TURN_CORNER_SIDE_THRESHOLD;
  bool leftPairPresent = sensorValues[SENSOR_SLIGHT_LEFT] >= TURN_CORNER_SIDE_THRESHOLD &&
                         sensorValues[SENSOR_LEFT] >= TURN_CORNER_SIDE_THRESHOLD;

  if (centerOnLine &&
      rightPairPresent &&
      !leftPairPresent &&
      rightScore >= TURN_CORNER_SCORE_MIN &&
      rightScore - leftScore >= TURN_CORNER_SCORE_MARGIN)
  {
    Serial.println("RETURN | Forced single-path RIGHT corner detected.");
    return 1;
  }

  if (centerOnLine &&
      leftPairPresent &&
      !rightPairPresent &&
      leftScore >= TURN_CORNER_SCORE_MIN &&
      leftScore - rightScore >= TURN_CORNER_SCORE_MARGIN)
  {
    Serial.println("RETURN | Forced single-path LEFT corner detected.");
    return -1;
  }

  return 0;
}

bool fieldNavigationActive()
{
  return !returnTunnelActive && (fieldModeActive || returnModeActive);
}

void resetLinePid()
{
  linePidIntegral = 0.0;
  linePidLastError = 0.0;
  linePidLastMs = 0;
  linePidHasLast = false;
}

bool computeLinePidError(float &error)
{
  const int sensorWeights[SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000};
  long weightedSum = 0;
  int signalTotal = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    int signal = sensorValues[i] > LINE_PID_SIGNAL_FLOOR
                     ? (int)(sensorValues[i] - LINE_PID_SIGNAL_FLOOR)
                     : 0;
    weightedSum += (long)signal * sensorWeights[i];
    signalTotal += signal;
  }

  if (signalTotal < LINE_PID_MIN_SIGNAL_TOTAL)
  {
    return false;
  }

  error = (float)weightedSum / (float)signalTotal;
  return true;
}

void driveLinePid(float error)
{
  unsigned long now = millis();
  float dtSec = 0.02;

  if (linePidHasLast && now > linePidLastMs)
  {
    dtSec = (float)(now - linePidLastMs) / 1000.0;
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
  if (linePidIntegral > LINE_PID_INTEGRAL_LIMIT)
  {
    linePidIntegral = LINE_PID_INTEGRAL_LIMIT;
  }
  else if (linePidIntegral < -LINE_PID_INTEGRAL_LIMIT)
  {
    linePidIntegral = -LINE_PID_INTEGRAL_LIMIT;
  }

  float derivative = linePidHasLast ? (error - linePidLastError) / dtSec : 0.0;
  float correctionFloat = LINE_PID_KP * error +
                          LINE_PID_KI * linePidIntegral +
                          LINE_PID_KD * derivative;
  int16_t maxCorrection = fieldNavigationActive()
                              ? FIELD_LINE_PID_MAX_CORRECTION
                              : (postTurnFineLineActive()
                                     ? POST_TURN_LINE_PID_MAX_CORRECTION
                                     : LINE_PID_MAX_CORRECTION);
  float limitedCorrection = constrain(
      correctionFloat,
      (float)-maxCorrection,
      (float)maxCorrection);
  int16_t correction = (int16_t)limitedCorrection;
  int16_t baseSpeed = postTurnFineLineActive()
                          ? (fieldNavigationActive()
                                 ? FIELD_POST_TURN_FINE_FORWARD_SPEED
                                 : POST_TURN_FINE_FORWARD_SPEED)
                          : (fieldNavigationActive() ? FIELD_FORWARD_SPEED : FORWARD_SPEED);
  int16_t leftSpeed = constrain((int16_t)(baseSpeed - correction), -MAX_SPEED, MAX_SPEED);
  int16_t rightSpeed = constrain((int16_t)(baseSpeed + correction), -MAX_SPEED, MAX_SPEED);

  if (error > 120.0)
  {
    lastDirection = -1;
  }
  else if (error < -120.0)
  {
    lastDirection = 1;
  }
  else
  {
    lastDirection = 0;
  }

  correctionDirection = 0;
  driveBySide(leftSpeed, rightSpeed);

  linePidLastError = error;
  linePidLastMs = now;
  linePidHasLast = true;
}

void linePidFollowStep()
{
  readSensors();
  printSensors("PID_LINE");

  if (SENSOR_DEBUG_MODE)
  {
    stopAllMotors();
    return;
  }

  if (airlockDirectTunnelEntryDue())
  {
    enterAirlockDirectTunnelMode("AIRLOCK_LINE_ALIGN_PID");
    return;
  }

  if (anySensorOnLine() && recordUphillTiltTransition("PID_TILT_PRETURN"))
  {
    stopAllMotors();
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  bool allowRightAngle = ENABLE_RIGHT_ANGLE_TURNS && millis() >= ignoreRightAngleUntilMs;
  if (allowRightAngle)
  {
    int turnDirection = returnModeActive
                            ? returnCornerTurnDirectionRequested()
                            : rightAngleTurnDirectionRequested();
    if (turnDirection < 0)
    {
      resetLinePid();
      handleLeftRightAngleDecision();
      return;
    }

    if (turnDirection > 0)
    {
      resetLinePid();
      handleRightRightAngleDecision();
      return;
    }
  }

  if (!anySensorOnLine())
  {
    resetLinePid();
    stopAllMotors();
    if (returnModeActive && returnPositionKnown && returnGridY > 4)
    {
      noLineTargetYawDeg = yawDeg;
      noLineYawTargetLocked = true;
      Serial.println("RETURN | Open field: hold heading until next RFID.");
      enterState(NO_LINE_IMU_FORWARD);
      return;
    }
    if (recordUphillTiltTransition("SENSE_TILT_LOST") ||
        recordUphillLineFlicker(false, "SENSE_LOST"))
    {
      enterState(NO_LINE_RETURN_YAW);
      return;
    }
    enterNoLineFindState();
    return;
  }

  float error = 0.0;
  if (!computeLinePidError(error))
  {
    resetLinePid();
    stopAllMotors();
    if (recordUphillTiltTransition("PID_TILT_LOST") ||
        recordUphillLineFlicker(false, "PID_LOST"))
    {
      enterState(NO_LINE_RETURN_YAW);
      return;
    }
    enterNoLineFindState();
    return;
  }

  if (recordUphillTiltTransition("PID_TILT"))
  {
    stopAllMotors();
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  driveLinePid(error);
}

uint8_t sensorsOnLineCount()
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

uint8_t logicalSensorsOnLineCount()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < LOGICAL_SENSOR_COUNT; i++)
  {
    if (sensorOnLine[LOGICAL_SENSOR_INDEXES[i]])
    {
      count++;
    }
  }
  return count;
}

bool lineOnSensorRange(uint8_t firstIndex, uint8_t lastIndex)
{
  if (firstIndex >= SENSOR_COUNT)
  {
    return false;
  }

  if (lastIndex >= SENSOR_COUNT)
  {
    lastIndex = SENSOR_COUNT - 1;
  }

  for (uint8_t i = firstIndex; i <= lastIndex; i++)
  {
    if (sensorOnLine[i])
    {
      return true;
    }
  }

  return false;
}

void handleRightRightAngleDecision()
{
  printSensorsNow(lastTurnDecisionWasTJunction ? "T_JUNCTION_SENSE" : "RIGHT_CORNER_SENSE");
  Serial.print(lastTurnDecisionWasTJunction ? "T_JUNCTION | Right priority routeTurn=" : "RIGHT_CORNER | Right turn routeTurn=");
  Serial.print(completedRouteTurns);
  Serial.println(" turn. Lead-in before IMU turn.");
  lastTurnDecisionWasTJunction = false;

  pendingTurnDirection = 1;
  lastDirection = 1;
  enterState(PRE_TURN_FORWARD_RIGHT);
}

void handleLeftRightAngleDecision()
{
  printSensorsNow("LEFT_CORNER_SENSE");
  Serial.print("LEFT_CORNER | Left turn routeTurn=");
  Serial.print(completedRouteTurns);
  Serial.println(" turn. Lead-in before IMU turn.");

  pendingTurnDirection = -1;
  lastDirection = -1;
  enterState(PRE_TURN_FORWARD_LEFT);
}

void prepareCorrection(int direction, bool strongCorrection)
{
  correctionDirection = direction;
  activeCorrectionPulseMs = postTurnFineLineActive()
                                ? POST_TURN_FINE_CORRECTION_PULSE_MS
                                : (strongCorrection ? STRONG_CORRECTION_PULSE_MS : CORRECTION_PULSE_MS);
  enterState(PRE_CORRECT_STOP);
}

void prepareRightAngleTurn(int direction)
{
  pendingTurnDirection = direction;
  enterState(direction < 0 ? PRE_TURN_FORWARD_LEFT : PRE_TURN_FORWARD_RIGHT);
}

void printSensors(const char *label)
{
  if (millis() - lastPrintMs < SERIAL_PRINT_INTERVAL_MS)
  {
    return;
  }

  printSensorsNow(label);
}

void printSensorsNow(const char *label)
{
  Serial.print(label);
  Serial.print(" ");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print("(D");
    Serial.print(SENSOR_PINS[i]);
    Serial.print(")=");
    Serial.print(sensorValues[i]);
    Serial.print(sensorOnLine[i] ? "*" : " ");
    Serial.print(" ");
  }

  Serial.print("| lastDir=");
  Serial.print(lastDirection);
  Serial.print(" corrDir=");
  Serial.print(correctionDirection);
  Serial.print(" routeTurn=");
  Serial.print(completedRouteTurns);
  Serial.print(" yaw=");
  Serial.println(yawDeg, 1);

  lastPrintMs = millis();
}

float readUltrasonicCm(UltrasonicSensor &sensor)
{
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.trigPin, LOW);

  unsigned long duration = pulseIn(sensor.echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0)
  {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}

void updateUltrasonic()
{
  if (millis() - lastUltrasonicReadMs < ULTRASONIC_SENSOR_GAP_MS)
  {
    return;
  }

  lastUltrasonicReadMs = millis();
  ultrasonicSensors[nextUltrasonicIndex].distanceCm = readUltrasonicCm(ultrasonicSensors[nextUltrasonicIndex]);
  nextUltrasonicIndex = (nextUltrasonicIndex + 1) % NUM_ULTRASONIC;
}

void printDistance(float distanceCm)
{
  if (distanceCm < 0.0)
  {
    Serial.print("NA");
  }
  else
  {
    Serial.print(distanceCm, 1);
  }
}

void printSensorStatus()
{
  if (millis() - lastSensorStatusMs < SENSOR_STATUS_INTERVAL_MS)
  {
    return;
  }

  lastSensorStatusMs = millis();

  Serial.print("STATUS | US Front=");
  printDistance(ultrasonicSensors[0].distanceCm);
  Serial.print("cm Left=");
  printDistance(ultrasonicSensors[1].distanceCm);
  Serial.print("cm Right=");
  printDistance(ultrasonicSensors[2].distanceCm);
  Serial.print("cm | Yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" | Kill=");
  Serial.print(killStopped ? "STOP" : "RUN");
  Serial.print(" | Planter=");
  Serial.print(planterStateName());
  Serial.print(" | State=");
  Serial.print(stateName(state));
  if (returnModeActive)
  {
    Serial.print(" | RETURN pos=");
    if (returnPositionKnown)
    {
      Serial.print(returnGridX);
      Serial.print(",");
      Serial.print(returnGridY);
    }
    else
    {
      Serial.print("?");
    }
    Serial.print(" heading=");
    Serial.print(gridHeading);
  }
  Serial.println();
}

void printStateThrottled()
{
  if (millis() - lastStatePrintMs < SERIAL_PRINT_INTERVAL_MS)
  {
    return;
  }

  lastStatePrintMs = millis();
  Serial.print("STATE | ");
  Serial.println(stateName(state));
}

bool obstacleAvoidanceAllowed()
{
  if (uphillTunnelPhaseReached())
  {
    return false;
  }

  switch (state)
  {
    case SENSE_LINE:
    case FORWARD_PULSE:
    case SENSOR_SETTLE:
    case PRE_CORRECT_STOP:
    case CORRECT_LEFT:
    case CORRECT_RIGHT:
    case AFTER_CORRECT_SETTLE:
    case NO_LINE_IMU_FORWARD:
    case WALL_FOLLOW:
      return true;

    case PRE_TURN_FORWARD_LEFT:
    case PRE_TURN_FORWARD_RIGHT:
    case TURN_LEFT_90:
    case TURN_RIGHT_90:
    case TURN_AROUND_180:
    case POST_TURN_FORWARD_VERIFY:
    case POST_TURN_BACK_UP:
    case AFTER_TURN_SETTLE:
    case FIND_LINE_SMALL_LEFT:
    case FIND_LINE_SMALL_RIGHT:
    case FIND_LINE_BIG_LEFT:
    case FIND_LINE_BIG_RIGHT:
    case NO_LINE_RETURN_YAW:
    case GRID_OBSTACLE_AVOID:
    case RFID_PAUSE:
    case LINE_LOST_STOP:
      return false;
  }

  return false;
}

bool distanceBlocked(float distanceCm)
{
  return distanceCm > 0.0 && distanceCm <= OBSTACLE_TURN_DISTANCE_CM;
}

bool gridObstacleAvoidanceAllowed()
{
  switch (state)
  {
    case SENSE_LINE:
    case FORWARD_PULSE:
    case SENSOR_SETTLE:
    case PRE_CORRECT_STOP:
    case CORRECT_LEFT:
    case CORRECT_RIGHT:
    case AFTER_CORRECT_SETTLE:
      return true;

    default:
      return false;
  }
}

bool gridObstacleDetectedAhead(float distanceCm)
{
  return distanceCm > 0.0 && distanceCm <= GRID_OBSTACLE_DETECT_CM;
}

int chooseFixedRecoveryTurnDirection()
{
  float leftCm = ultrasonicSensors[1].distanceCm;
  float rightCm = ultrasonicSensors[2].distanceCm;
  bool leftValid = leftCm > 0.0;
  bool rightValid = rightCm > 0.0;

  if (leftValid && rightValid && fabs(leftCm - rightCm) >= 3.0)
  {
    return leftCm > rightCm ? -1 : 1;
  }

  if (leftValid != rightValid)
  {
    return leftValid ? -1 : 1;
  }

  int direction = recoveryDirectionFallback;
  recoveryDirectionFallback = -recoveryDirectionFallback;
  return direction;
}

void startFixedRecovery(const char *reason, int requestedDirection)
{
  stopAllMotors();
  frontObstacleHolding = false;
  fixedRecoveryActive = true;
  fixedRecoveryTurnDirection = requestedDirection == 0
                                   ? chooseFixedRecoveryTurnDirection()
                                   : (requestedDirection < 0 ? -1 : 1);
  lastTurnDirection = fixedRecoveryTurnDirection;

  Serial.print("RECOVERY_FIXED | reason=");
  Serial.print(reason);
  Serial.print(" backup=");
  Serial.print(POST_TURN_BACKUP_MS);
  Serial.print("ms uturn=");
  Serial.print(fixedRecoveryTurnDirection < 0 ? "LEFT" : "RIGHT");
  Serial.println(" then offline/no-line navigation.");

  enterState(POST_TURN_BACK_UP);
}

bool handleFrontObstacle()
{
  if (!USE_FRONT_OBSTACLE_AVOIDANCE)
  {
    return false;
  }
  if (millis() < ignoreFrontObstacleUntilMs)
  {
    frontObstacleHolding = false;
    return false;
  }

  if (USE_GRID_OBSTACLE_AVOIDANCE &&
      gridObstacleAvoidanceAllowed() &&
      gridObstacleDetectedAhead(ultrasonicSensors[0].distanceCm) &&
      (gridObstacleLastCompleteMs == 0 || millis() - gridObstacleLastCompleteMs >= GRID_OBSTACLE_COOLDOWN_MS))
  {
    startGridObstacleAvoidance();
    return true;
  }

  if (distanceBlocked(ultrasonicSensors[0].distanceCm))
  {
    stopAllMotors();
    bool newlyBlocked = !frontObstacleHolding;

    if (newlyBlocked)
    {
      frontObstacleHoldStartMs = millis();
    }

    frontObstacleHolding = true;
    if (millis() - frontObstacleHoldStartMs >= FRONT_OBSTACLE_RECOVERY_MS)
    {
      startFixedRecovery("front_obstacle_timeout");
      return true;
    }

    if (newlyBlocked || millis() - lastSensorStatusMs >= SENSOR_STATUS_INTERVAL_MS)
    {
      Serial.print("OBSTACLE_STOP | Front=");
      printDistance(ultrasonicSensors[0].distanceCm);
      Serial.print("cm <= ");
      Serial.print(OBSTACLE_TURN_DISTANCE_CM, 1);
      Serial.println("cm. Holding until clear.");
    }

    return true;
  }

  if (frontObstacleHolding)
  {
    unsigned long pausedMs = millis() - frontObstacleHoldStartMs;
    stateStartMs += pausedMs;
    lastUpdateMicros = micros();

    Serial.print("OBSTACLE_CLEAR | Front=");
    printDistance(ultrasonicSensors[0].distanceCm);
    Serial.print("cm. Resuming after ");
    Serial.print(pausedMs);
    Serial.println("ms hold.");
    frontObstacleHolding = false;
  }

  return false;
}

void enterState(RobotState nextState)
{
  state = nextState;
  stateStartMs = millis();

  switch (state)
  {
    case SENSE_LINE:
      break;

    case FORWARD_PULSE:
      if (USE_LINE_PID)
      {
        resetLinePid();
      }
      printStateThrottled();
      break;

    case SENSOR_SETTLE:
      stopAllMotors();
      break;

    case PRE_CORRECT_STOP:
      printStateThrottled();
      stopAllMotors();
      break;

    case CORRECT_LEFT:
      printStateThrottled();
      break;

    case CORRECT_RIGHT:
      printStateThrottled();
      break;

    case AFTER_CORRECT_SETTLE:
      stopAllMotors();
      break;

    case PRE_TURN_FORWARD_LEFT:
      printStateThrottled();
      Serial.print("PRE_TURN | target cm=");
      Serial.print(activePreTurnForwardLeftCm(), 1);
      Serial.print(" ms=");
      Serial.print(activePreTurnForwardLeftMs());
      Serial.print(" routeTurn=");
      Serial.println(completedRouteTurns);
      break;

    case PRE_TURN_FORWARD_RIGHT:
      printStateThrottled();
      Serial.print("PRE_TURN | target cm=");
      Serial.print(PRE_TURN_FORWARD_RIGHT_CM, 1);
      Serial.print(" ms=");
      Serial.println(PRE_TURN_FORWARD_RIGHT_MS);
      break;

    case TURN_LEFT_90:
      lastTurnDirection = -1;
      beginRightAngleTurn(-1);
      printStateThrottled();
      break;

    case TURN_RIGHT_90:
      lastTurnDirection = 1;
      beginRightAngleTurn(1);
      printStateThrottled();
      break;

    case TURN_AROUND_180:
      beginTurnAround180();
      printStateThrottled();
      break;

    case POST_TURN_FORWARD_VERIFY:
      resetLinePid();
      postTurnStartTiltDeg = calibratedTiltMagnitudeDeg();
      postTurnMaxTiltDeltaDeg = 0.0;
      copyEncoderCounts(postTurnStartEncoderCounts);
      Serial.print("POST_TURN | verify forward ms=");
      Serial.print(POST_TURN_VERIFY_MS);
      Serial.print(" frontBlock=");
      Serial.print(POST_TURN_FRONT_BLOCK_CM, 1);
      Serial.print("cm minTiltDelta=");
      Serial.print(POST_TURN_MIN_TILT_DELTA_DEG, 1);
      Serial.println("deg");
      break;

    case POST_TURN_BACK_UP:
      stopAllMotors();
      Serial.print("POST_TURN | backup ms=");
      Serial.println(POST_TURN_BACKUP_MS);
      break;

    case AFTER_TURN_SETTLE:
      stopAllMotors();
      break;

    case FIND_LINE_SMALL_LEFT:
      findLineStartYawDeg = yawDeg;
      printStateThrottled();
      break;

    case FIND_LINE_SMALL_RIGHT:
      findLineStartYawDeg = yawDeg;
      printStateThrottled();
      break;

    case FIND_LINE_BIG_LEFT:
      findLineStartYawDeg = yawDeg;
      printStateThrottled();
      break;

    case FIND_LINE_BIG_RIGHT:
      findLineStartYawDeg = yawDeg;
      printStateThrottled();
      break;

    case NO_LINE_RETURN_YAW:
      printStateThrottled();
      Serial.print("NO_LINE | Return yaw target=");
      Serial.print(noLineTargetYawDeg, 1);
      Serial.print(" tol=");
      Serial.print(NO_LINE_RETURN_YAW_TOLERANCE_DEG, 1);
      Serial.println("deg before tunnel/wall follow.");
      break;

    case NO_LINE_IMU_FORWARD:
      if (!noLineYawTargetLocked)
      {
        noLineTargetYawDeg = yawDeg;
      }
      Serial.print("NO_LINE | Holding yaw ");
      Serial.println(noLineTargetYawDeg, 1);
      printStateThrottled();
      break;

    case WALL_FOLLOW:
      wallPdLastError = 0.0;
      wallPdLastMs = millis();
      wallPdHasLast = false;
      wallAppliedBaseSpeed = WALL_FORWARD_SPEED;
      wallSpeedRampLastMs = millis();
      wallLineReacquireSinceMs = 0;
      copyEncoderCounts(wallFollowLastEncoderCounts);
      wallFollowLastDebugMs = 0;
      printStateThrottled();
      break;

    case GRID_OBSTACLE_AVOID:
      printStateThrottled();
      break;

    case RFID_PAUSE:
      stopAllMotors();
      printStateThrottled();
      Serial.print("RFID | Pause ");
      Serial.print(RFID_PAUSE_MS);
      Serial.print("ms, plant=");
      Serial.print(rfidPauseShouldPlant ? "YES" : "WAIT/NO");
      if (serverRfidRequest != SERVER_RFID_NONE)
      {
        Serial.print(" serverWait=");
        Serial.print(serverRfidRequest == SERVER_RFID_WAIT_FERTILITY ? "FERTILITY" : "AIRLOCK");
        Serial.print(" timeout=");
        Serial.print(SERVER_REPLY_TIMEOUT_MS);
        Serial.print("ms");
      }
      Serial.println();
      break;

    case LINE_LOST_STOP:
      printStateThrottled();
      stopAllMotors();
      break;
  }
}

void startAirlockUphillRouteTracking(const char *source)
{
  if (airlockRouteTracking)
  {
    return;
  }

  airlockRouteTracking = true;
  airlockRouteTurns = 0;
  Serial.print("ROUTE | Airlock/uphill tracking armed by ");
  Serial.print(source);
  Serial.println(". Expect LEFT then RIGHT before slope.");
}

void recordAirlockRouteTurn(int direction)
{
  if (!airlockRouteTracking || airlockRouteTurns >= AIRLOCK_UPHILL_ROUTE_TURNS)
  {
    return;
  }

  if (airlockRouteTurns == 0)
  {
    if (direction < 0)
    {
      airlockRouteTurns = 1;
      Serial.println("ROUTE | Airlock turn 1/2 LEFT complete.");
    }
    return;
  }

  if (airlockRouteTurns == 1 && direction > 0)
  {
    airlockRouteTurns = 2;
    noLineTargetYawDeg = yawDeg;
    noLineYawTargetLocked = true;
    airlockDirectTunnelEntryArmed = true;
    airlockDirectTunnelEntryStartMs = millis();
    Serial.println("ROUTE | Airlock turn 2/2 RIGHT complete. Armed direct tunnel entry after line align.");
  }
}

bool uphillTunnelPhaseReached()
{
  return airlockRouteTurns >= AIRLOCK_UPHILL_ROUTE_TURNS;
}

bool airlockDirectTunnelEntryDue()
{
  return airlockDirectTunnelEntryArmed &&
         uphillTunnelPhaseReached() &&
         millis() - airlockDirectTunnelEntryStartMs >= AIRLOCK_UPHILL_LINE_ALIGN_MS;
}

void enterAirlockDirectTunnelMode(const char *source)
{
  airlockDirectTunnelEntryArmed = false;
  uphillLineFlickerYawDeg = yawDeg;
  noLineTargetYawDeg = yawDeg;
  noLineYawTargetLocked = true;
  stopAllMotors();
  Serial.print("ROUTE | Direct tunnel entry from ");
  Serial.print(source);
  Serial.print(" yaw=");
  Serial.println(noLineTargetYawDeg, 1);
  enterUphillFlickerBypass(source);
  enterState(wallFollowingAllowed() ? WALL_FOLLOW : NO_LINE_IMU_FORWARD);
}

bool wallFollowingAllowed()
{
  return USE_WALL_FOLLOWING && (uphillTunnelPhaseReached() || WALL_FOLLOW_TEST_MODE);
}

bool inUphillLineFlickerGuardZone()
{
  return ENABLE_UPHILL_LINE_FLICKER_GUARD &&
         uphillTunnelPhaseReached();
}

bool uphillLineIgnoreActive()
{
  return ENABLE_UPHILL_LINE_FLICKER_GUARD &&
         millis() < uphillLineIgnoreUntilMs;
}

void enterUphillFlickerBypass(const char *source)
{
  unsigned long now = millis();
  unsigned long ignoreUntil = now + UPHILL_LINE_IGNORE_MS;

  uphillLineIgnoreUntilMs = ignoreUntil;
  noLineTargetYawDeg = uphillLineFlickerYawDeg;
  noLineYawTargetLocked = true;
  if (ignoreRightAngleUntilMs < ignoreUntil)
  {
    ignoreRightAngleUntilMs = ignoreUntil;
  }

  uphillLineFlickerWindowStartMs = now;
  uphillLineFlickerTransitions = 0;
  uphillLineFlickerHasLast = false;
  resetLinePid();

  Serial.print("UPHILL_GUARD | flicker source=");
  Serial.print(source);
  Serial.print(" targetYaw=");
  Serial.print(noLineTargetYawDeg, 1);
  Serial.print(" ignoreLineMs=");
  Serial.println(UPHILL_LINE_IGNORE_MS);
}

bool recordUphillLineFlicker(bool lineSeen, const char *source)
{
  if (!inUphillLineFlickerGuardZone())
  {
    uphillLineFlickerHasLast = false;
    uphillLineFlickerTransitions = 0;
    uphillLineFlickerWindowStartMs = 0;
    return false;
  }

  unsigned long now = millis();
  if (!lineSeen && wallFollowingAllowed() && sideWallCloseForNoLineEntry())
  {
    uphillLineFlickerYawDeg = yawDeg;
    enterUphillFlickerBypass(source);
    return true;
  }

  if (!uphillLineFlickerHasLast ||
      uphillLineFlickerWindowStartMs == 0 ||
      now - uphillLineFlickerWindowStartMs > UPHILL_LINE_FLICKER_WINDOW_MS)
  {
    uphillLineFlickerWindowStartMs = now;
    uphillLineFlickerTransitions = 0;
    uphillLineFlickerHasLast = true;
    uphillLineFlickerLastSeen = lineSeen;
    if (!lineSeen)
    {
      uphillLineFlickerYawDeg = yawDeg;
    }
    return false;
  }

  if (lineSeen != uphillLineFlickerLastSeen)
  {
    uphillLineFlickerTransitions++;
    uphillLineFlickerLastSeen = lineSeen;
    if (!lineSeen)
    {
      uphillLineFlickerYawDeg = yawDeg;
    }
  }

  if (uphillLineFlickerTransitions >= UPHILL_LINE_FLICKER_TRIGGER_TRANSITIONS)
  {
    enterUphillFlickerBypass(source);
    return true;
  }

  return false;
}

bool recordUphillTiltTransition(const char *source)
{
  if (!ENABLE_UPHILL_TILT_GUARD ||
      !inUphillLineFlickerGuardZone() ||
      uphillLineIgnoreActive())
  {
    uphillTiltGuardStartMs = 0;
    return false;
  }

  unsigned long now = millis();
  if (uphillTiltGuardLastTriggerMs != 0 &&
      now - uphillTiltGuardLastTriggerMs < UPHILL_TILT_GUARD_COOLDOWN_MS)
  {
    return false;
  }

  float pitchDeltaDeg = calibratedPitchDeltaDeg();
  float rollDeltaDeg = calibratedRollDeltaDeg();
  float tiltDeg = calibratedTiltMagnitudeDeg();
  bool tilted =
      tiltDeg >= UPHILL_TILT_TRIGGER_DEG ||
      fabs(rollDeltaDeg) >= UPHILL_ROLL_TRIGGER_DEG ||
      fabs(pitchDeltaDeg) >= UPHILL_PITCH_TRIGGER_DEG;

  if (!tilted)
  {
    uphillTiltGuardStartMs = 0;
    return false;
  }

  if (uphillTiltGuardStartMs == 0)
  {
    uphillTiltGuardStartMs = now;
    uphillTiltGuardYawDeg = yawDeg;
    return false;
  }

  if (now - uphillTiltGuardStartMs < UPHILL_TILT_GUARD_MIN_MS)
  {
    return false;
  }

  uphillLineFlickerYawDeg = uphillTiltGuardYawDeg;
  uphillTiltGuardLastTriggerMs = now;
  uphillTiltGuardStartMs = 0;
  enterUphillFlickerBypass(source);

  Serial.print("UPHILL_GUARD | tilt dPitch=");
  Serial.print(pitchDeltaDeg, 1);
  Serial.print(" dRoll=");
  Serial.print(rollDeltaDeg, 1);
  Serial.print(" dTilt=");
  Serial.println(tiltDeg, 1);
  return true;
}

void senseLineStep()
{
  readSensors();
  printSensors(SENSOR_DEBUG_MODE ? "QTR_DEBUG" : "SENSE");

  if (SENSOR_DEBUG_MODE)
  {
    stopAllMotors();
    return;
  }

  if (airlockDirectTunnelEntryDue())
  {
    enterAirlockDirectTunnelMode("AIRLOCK_LINE_ALIGN_SENSE");
    return;
  }

  if (anySensorOnLine() && recordUphillTiltTransition("SENSE_TILT"))
  {
    stopAllMotors();
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  // Logical 5-point order from the 9-output board:
  // OUT1 right, OUT3 slight right, OUT5 middle, OUT7 slight left, OUT9 left.
  bool allowRightAngle = ENABLE_RIGHT_ANGLE_TURNS && millis() >= ignoreRightAngleUntilMs;
  bool middleBandDetected = middleBandOnLine();

  if (allowRightAngle)
  {
    int turnDirection = returnModeActive
                            ? returnCornerTurnDirectionRequested()
                            : rightAngleTurnDirectionRequested();
    if (turnDirection < 0)
    {
      handleLeftRightAngleDecision();
      return;
    }

    if (turnDirection > 0)
    {
      handleRightRightAngleDecision();
      return;
    }
  }

  if (USE_LINE_PID && anySensorOnLine())
  {
    enterState(FORWARD_PULSE);
    return;
  }

  if (middleBandDetected)
  {
    lastDirection = 0;
    correctionDirection = 0;
    activeCorrectionPulseMs = CORRECTION_PULSE_MS;
    enterState(FORWARD_PULSE);
    return;
  }

  bool leftNeedsCorrection = leftCorrectionRequested();
  bool rightNeedsCorrection = rightCorrectionRequested();

  if (leftNeedsCorrection && !rightNeedsCorrection)
  {
    lastDirection = -1;

    if (ENABLE_LINE_CORRECTION)
    {
      prepareCorrection(-1, leftStrongCorrectionRequested());
    }
    else
    {
      stopAllMotors();
    }
    return;
  }

  if (rightNeedsCorrection && !leftNeedsCorrection)
  {
    lastDirection = 1;

    if (ENABLE_LINE_CORRECTION)
    {
      prepareCorrection(1, rightStrongCorrectionRequested());
    }
    else
    {
      stopAllMotors();
    }
    return;
  }

  if (sensorOnLine[SENSOR_LEFT])
  {
    lastDirection = -1;
    stopAllMotors();
    return;
  }

  if (sensorOnLine[SENSOR_RIGHT])
  {
    lastDirection = 1;
    stopAllMotors();
    return;
  }

  if (wallFollowingAllowed())
  {
    enterState(WALL_FOLLOW);
  }
  else
  {
    enterNoLineFindState();
  }
}

void correctionStep()
{
  readSensors();
  printSensors(correctionDirection < 0 ? "CORRECT_LEFT" : "CORRECT_RIGHT");
  int16_t correctionSpeed = postTurnFineLineActive()
                                ? POST_TURN_FINE_CORRECTION_SPEED
                                : max(PIVOT_SPEED, MIN_CORRECTION_SPEED);
  if (fieldNavigationActive())
  {
    correctionSpeed = max((int16_t)360, scaledSpeed(correctionSpeed, FIELD_TURN_SPEED_PERCENT));
  }

  if (millis() - stateStartMs >= activeCorrectionPulseMs)
  {
    stopAllMotors();
    enterState(AFTER_CORRECT_SETTLE);
    return;
  }

  if (correctionDirection < 0)
  {
    pivotLeft(correctionSpeed);
  }
  else if (correctionDirection > 0)
  {
    pivotRight(correctionSpeed);
  }
  else
  {
    stopAllMotors();
  }
}

const char *rightAnglePhaseName(RightAnglePhase phase)
{
  switch (phase)
  {
    case RIGHT_ANGLE_APPROACH: return "APPROACH";
    case RIGHT_ANGLE_SEGMENT_STOP: return "SEGMENT_STOP";
    case RIGHT_ANGLE_RELIEF_FORWARD: return "RELIEF_FORWARD";
    case RIGHT_ANGLE_RELIEF_BACKWARD: return "RELIEF_BACKWARD";
    case RIGHT_ANGLE_SETTLE: return "SETTLE";
    case RIGHT_ANGLE_FINE_CORRECT: return "FINE_CORRECT";
  }

  return "UNKNOWN";
}

float rightAngleTurnErrorDeg()
{
  return activeTurnTargetDeg - rightAngleTurnDeg();
}

float limitedRightAngleSegmentTarget(float currentDeg)
{
  float target = currentDeg + RIGHT_ANGLE_SEGMENT_DEG;
  if (target > activeTurnTargetDeg)
  {
    target = activeTurnTargetDeg;
  }

  return target;
}

float tiltMagnitudeDeg()
{
  return sqrt(pitchDeg * pitchDeg + rollDeg * rollDeg);
}

float calibratedPitchDeltaDeg()
{
  return tiltBaselineCaptured ? pitchDeg - tiltBaselinePitchDeg : pitchDeg;
}

float calibratedRollDeltaDeg()
{
  return tiltBaselineCaptured ? normalizedAngleDeltaDeg(rollDeg, tiltBaselineRollDeg) : rollDeg;
}

float calibratedTiltMagnitudeDeg()
{
  float pitchDelta = calibratedPitchDeltaDeg();
  float rollDelta = calibratedRollDeltaDeg();
  return sqrt(pitchDelta * pitchDelta + rollDelta * rollDelta);
}

int16_t rightAngleSpeedForError(float remainingDeg)
{
  int16_t speed = RIGHT_ANGLE_FINE_SPEED;
  if (remainingDeg > RIGHT_ANGLE_MID_REMAINING_DEG)
  {
    speed = RIGHT_ANGLE_FAST_SPEED;
  }
  else if (remainingDeg > RIGHT_ANGLE_FINE_REMAINING_DEG)
  {
    speed = RIGHT_ANGLE_MID_SPEED;
  }

  return fieldNavigationActive()
             ? max((int16_t)300, scaledSpeed(speed, FIELD_TURN_SPEED_PERCENT))
             : speed;
}

void driveTurnDirection(int direction, int16_t speed)
{
  if (direction < 0)
  {
    balancedPivot(direction, speed);
  }
  else if (direction > 0)
  {
    balancedPivot(direction, speed);
  }
  else
  {
    stopAllMotors();
  }
}

void enterRightAnglePhase(RightAnglePhase phase)
{
  rightAnglePhase = phase;
  rightAnglePhaseStartMs = millis();

  Serial.print("TURN_PHASE | ");
  Serial.println(rightAnglePhaseName(rightAnglePhase));
}

void beginRightAngleTurn(int direction, float targetDeg)
{
  activeTurnDirection = direction < 0 ? -1 : 1;
  fineTurnDirection = activeTurnDirection;
  activeTurnTargetDeg = targetDeg;
  turnLineAbortExtraActive = false;
  turnStartYawDeg = yawDeg;
  rightAngleSegmentTargetDeg = limitedRightAngleSegmentTarget(rightAngleTurnDeg());
  rightAnglePhaseStartMs = millis();
  nextTurnReliefForward = true;
  rightAnglePhase = RIGHT_ANGLE_APPROACH;
  resetTurnEncoderBalance();

  Serial.print("TURN | segmented ");
  if (activeTurnTargetDeg >= UTURN_TARGET_DEG - 1.0)
  {
    Serial.print(activeTurnDirection < 0 ? "LEFT_180" : "RIGHT_180");
  }
  else
  {
    Serial.print(activeTurnDirection < 0 ? "LEFT_90" : "RIGHT_90");
  }
  Serial.print(" target=");
  Serial.print(activeTurnTargetDeg, 1);
  Serial.print(" segment=");
  Serial.print(RIGHT_ANGLE_SEGMENT_DEG, 1);
  Serial.print(" relief=");
  Serial.print(RIGHT_ANGLE_RELIEF_PULSE_MS);
  Serial.print("ms alternating forward/back encBalance=");
  Serial.println(ENABLE_ENCODER_TURN_BALANCE ? "ON" : "OFF");
}

void beginTurnAround180()
{
  int direction = fixedRecoveryTurnDirection != 0
                      ? fixedRecoveryTurnDirection
                      : (lastTurnDirection < 0 ? 1 : -1);
  fixedRecoveryTurnDirection = 0;
  beginRightAngleTurn(direction, UTURN_TARGET_DEG);
  Serial.println("RECOVERY | Backed up. Starting 180-degree turn.");
}

void finishRightAngleTurn(const char *reason)
{
  stopAllMotors();
  int completedDirection = activeTurnDirection != 0 ? activeTurnDirection : lastTurnDirection;
  bool completedUTurn = activeTurnTargetDeg >= UTURN_TARGET_DEG - 1.0;
  if (completedUTurn)
  {
    gridHeading = (gridHeading + 2) % 4;
  }
  else if (completedDirection > 0)
  {
    gridHeading = (gridHeading + 1) % 4;
  }
  else if (completedDirection < 0)
  {
    gridHeading = (gridHeading + 3) % 4;
  }

  Serial.print("TURN | ");
  Serial.print(reason);
  Serial.print(" delta=");
  Serial.print(rightAngleTurnDeg(), 2);
  Serial.print(" error=");
  Serial.print(rightAngleTurnErrorDeg(), 2);
  Serial.print(" heading=");
  Serial.println(gridHeading);
  lastTurnDirection = completedDirection;

  if (completedUTurn)
  {
    activeTurnDirection = 0;
    pendingTurnDirection = 0;
    turnLineAbortExtraActive = false;
    if (fixedRecoveryActive)
    {
      fixedRecoveryActive = false;
      noLineTargetYawDeg = yawDeg;
      noLineYawTargetLocked = true;
      ignoreRightAngleUntilMs = millis() + 800;
      Serial.println("RECOVERY_FIXED | U-turn complete. Entering offline/no-line navigation.");
      enterState(wallFollowingAllowed() ? WALL_FOLLOW : NO_LINE_IMU_FORWARD);
      return;
    }
    enterState(AFTER_TURN_SETTLE);
    return;
  }

  completedRouteTurns++;
  recordAirlockRouteTurn(lastTurnDirection);

  if (turnLineAbortExtraActive)
  {
    turnLineAbortExtraActive = false;
    activeTurnDirection = 0;
    pendingTurnDirection = 0;
    ignoreRightAngleUntilMs = millis() + 600;
    enterState(SENSE_LINE);
    return;
  }

  enterState(POST_TURN_FORWARD_VERIFY);
}

bool baseUphillTurnLineAbortAllowed()
{
  return ENABLE_TURN_LINE_ABORT &&
         !turnLineAbortExtraActive &&
         activeTurnTargetDeg < UTURN_TARGET_DEG - 1.0 &&
         completedRouteTurns < BASE_UPHILL_ROUTE_TURN_COUNT &&
         rightAngleTurnDeg() >= TURN_LINE_ABORT_MIN_DEG;
}

void beginRightAngleTurnLineExtra()
{
  float currentDeg = rightAngleTurnDeg();
  float originalTargetDeg = activeTurnTargetDeg;
  float extraTargetDeg = currentDeg + TURN_LINE_ABORT_EXTRA_DEG;

  if (extraTargetDeg > originalTargetDeg)
  {
    extraTargetDeg = originalTargetDeg;
  }

  activeTurnTargetDeg = extraTargetDeg;
  rightAngleSegmentTargetDeg = limitedRightAngleSegmentTarget(currentDeg);
  turnLineAbortExtraActive = true;

  Serial.print("TURN | line_found extra_turn=");
  Serial.print(TURN_LINE_ABORT_EXTRA_DEG, 1);
  Serial.print("deg delta=");
  Serial.print(currentDeg, 2);
  Serial.print(" target=");
  Serial.print(activeTurnTargetDeg, 2);
  Serial.print(" routeIndex=");
  Serial.println(completedRouteTurns);
}

void startPostTurnRecovery(const char *reason)
{
  stopAllMotors();

  Serial.print("POST_TURN | recovery reason=");
  Serial.print(reason);
  Serial.print(" front=");
  printDistance(ultrasonicSensors[0].distanceCm);
  Serial.print("cm tiltDelta=");
  Serial.print(postTurnMaxTiltDeltaDeg, 2);
  Serial.print(" encMotion=");
  Serial.println(encoderMotionSince(postTurnStartEncoderCounts));

  enterState(POST_TURN_BACK_UP);
}

void printTurnStatus(const char *label, int16_t speedCommand)
{
  if (millis() - lastPrintMs < SERIAL_PRINT_INTERVAL_MS)
  {
    return;
  }

  lastPrintMs = millis();
  Serial.print("TURN | ");
  Serial.print(label);
  Serial.print(" phase=");
  Serial.print(rightAnglePhaseName(rightAnglePhase));
  Serial.print(" yaw=");
  Serial.print(yawDeg, 1);
  Serial.print(" delta=");
  Serial.print(rightAngleTurnDeg(), 1);
  Serial.print("/");
  Serial.print(activeTurnTargetDeg, 1);
  Serial.print(" error=");
  Serial.print(rightAngleTurnErrorDeg(), 1);
  Serial.print(" speed=");
  Serial.print(speedCommand);
  Serial.print(" segmentTarget=");
  Serial.print(rightAngleSegmentTargetDeg, 1);
  if (ENABLE_ENCODER_TURN_BALANCE)
  {
    Serial.print(" trim RF=");
    Serial.print(turnBalanceAppliedTrim[0]);
    Serial.print(" RR=");
    Serial.print(turnBalanceAppliedTrim[1]);
    Serial.print(" LR=");
    Serial.print(turnBalanceAppliedTrim[2]);
    Serial.print(" LF=");
    Serial.print(turnBalanceAppliedTrim[3]);
    Serial.print(" enc RF=");
    Serial.print(turnBalanceDeltaAbs[0]);
    Serial.print(" RR=");
    Serial.print(turnBalanceDeltaAbs[1]);
    Serial.print(" LR=");
    Serial.print(turnBalanceDeltaAbs[2]);
    Serial.print(" LF=");
    Serial.print(turnBalanceDeltaAbs[3]);
    Serial.print(" active=");
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
      Serial.print(turnBalanceEncoderActive[i] ? "1" : "0");
    }
    Serial.print(turnBalanceHasSample ? "" : " encWait");
  }
  Serial.println();
}

void rightAngleTurnStep(bool turnLeft)
{
  int direction = activeTurnDirection != 0 ? activeTurnDirection : (turnLeft ? -1 : 1);
  unsigned long now = millis();
  float error = rightAngleTurnErrorDeg();
  int16_t speedCommand = 0;
  unsigned long turnTimeoutMs = activeTurnTargetDeg >= UTURN_TARGET_DEG - 1.0 ? UTURN_TIMEOUT_MS : RIGHT_ANGLE_TIMEOUT_MS;

  if (baseUphillTurnLineAbortAllowed())
  {
    readSensors();
    if (anySensorOnLine())
    {
      beginRightAngleTurnLineExtra();
      error = rightAngleTurnErrorDeg();
    }
  }

  if (now - stateStartMs >= turnTimeoutMs)
  {
    Serial.print("TURN | IMU target not reached. delta=");
    Serial.println(rightAngleTurnDeg(), 1);
    stopAllMotors();
    pendingTurnDirection = 0;
    activeTurnDirection = 0;
    turnLineAbortExtraActive = false;
    if (fixedRecoveryActive)
    {
      fixedRecoveryActive = false;
      noLineTargetYawDeg = yawDeg;
      noLineYawTargetLocked = true;
      Serial.println("RECOVERY_FIXED | Recovery turn timed out. Continue cautiously in no-line mode.");
      enterState(wallFollowingAllowed() ? WALL_FOLLOW : NO_LINE_IMU_FORWARD);
    }
    else
    {
      startFixedRecovery("turn_timeout");
    }
    return;
  }

  switch (rightAnglePhase)
  {
    case RIGHT_ANGLE_APPROACH:
      if (error <= 0.0)
      {
        stopAllMotors();
        enterRightAnglePhase(RIGHT_ANGLE_SETTLE);
      }
      else if (rightAngleTurnDeg() >= rightAngleSegmentTargetDeg && error > RIGHT_ANGLE_FINE_REMAINING_DEG)
      {
        stopAllMotors();
        enterRightAnglePhase(RIGHT_ANGLE_SEGMENT_STOP);
      }
      else
      {
        speedCommand = rightAngleSpeedForError(error);
        driveTurnDirection(direction, speedCommand);
      }
      break;

    case RIGHT_ANGLE_SEGMENT_STOP:
      stopAllMotors();
      if (now - rightAnglePhaseStartMs >= RIGHT_ANGLE_SEGMENT_STOP_MS)
      {
        error = rightAngleTurnErrorDeg();

        if (error <= 0.0 || fabs(error) <= RIGHT_ANGLE_TOLERANCE_DEG)
        {
          enterRightAnglePhase(RIGHT_ANGLE_SETTLE);
        }
        else if (error <= RIGHT_ANGLE_FINE_REMAINING_DEG)
        {
          rightAngleSegmentTargetDeg = limitedRightAngleSegmentTarget(rightAngleTurnDeg());
          enterRightAnglePhase(RIGHT_ANGLE_APPROACH);
        }
        else if (nextTurnReliefForward)
        {
          enterRightAnglePhase(RIGHT_ANGLE_RELIEF_FORWARD);
        }
        else
        {
          enterRightAnglePhase(RIGHT_ANGLE_RELIEF_BACKWARD);
        }
      }
      break;

    case RIGHT_ANGLE_RELIEF_FORWARD:
      speedCommand = fieldNavigationActive()
                         ? max((int16_t)300, scaledSpeed(RIGHT_ANGLE_RELIEF_SPEED, FIELD_TURN_SPEED_PERCENT))
                         : RIGHT_ANGLE_RELIEF_SPEED;
      driveForward(speedCommand);
      if (now - rightAnglePhaseStartMs >= RIGHT_ANGLE_RELIEF_PULSE_MS)
      {
        stopAllMotors();
        nextTurnReliefForward = false;
        rightAngleSegmentTargetDeg = limitedRightAngleSegmentTarget(rightAngleTurnDeg());
        enterRightAnglePhase(RIGHT_ANGLE_APPROACH);
      }
      break;

    case RIGHT_ANGLE_RELIEF_BACKWARD:
      speedCommand = fieldNavigationActive()
                         ? -max((int16_t)300, scaledSpeed(RIGHT_ANGLE_RELIEF_SPEED, FIELD_TURN_SPEED_PERCENT))
                         : -RIGHT_ANGLE_RELIEF_SPEED;
      driveForward(speedCommand);
      if (now - rightAnglePhaseStartMs >= RIGHT_ANGLE_RELIEF_PULSE_MS)
      {
        stopAllMotors();
        nextTurnReliefForward = true;
        rightAngleSegmentTargetDeg = limitedRightAngleSegmentTarget(rightAngleTurnDeg());
        enterRightAnglePhase(RIGHT_ANGLE_APPROACH);
      }
      break;

    case RIGHT_ANGLE_SETTLE:
      stopAllMotors();
      if (now - rightAnglePhaseStartMs >= RIGHT_ANGLE_SETTLE_MS)
      {
        error = rightAngleTurnErrorDeg();

        if (fabs(error) <= RIGHT_ANGLE_TOLERANCE_DEG)
        {
          activeTurnDirection = 0;
          finishRightAngleTurn("target_reached");
        }
        else
        {
          fineTurnDirection = error > 0.0 ? direction : -direction;
          enterRightAnglePhase(RIGHT_ANGLE_FINE_CORRECT);
        }
      }
      break;

    case RIGHT_ANGLE_FINE_CORRECT:
      if (fabs(error) <= RIGHT_ANGLE_TOLERANCE_DEG)
      {
        activeTurnDirection = 0;
        finishRightAngleTurn("fine_done");
      }
      else if (fineTurnDirection == direction && error < -RIGHT_ANGLE_TOLERANCE_DEG)
      {
        stopAllMotors();
        enterRightAnglePhase(RIGHT_ANGLE_SETTLE);
      }
      else if (fineTurnDirection != direction && error > RIGHT_ANGLE_TOLERANCE_DEG)
      {
        stopAllMotors();
        enterRightAnglePhase(RIGHT_ANGLE_SETTLE);
      }
      else
      {
        speedCommand = rightAngleSpeedForError(fabs(error));
        driveTurnDirection(fineTurnDirection, speedCommand);
      }
      break;
  }

  const char *label = activeTurnTargetDeg >= UTURN_TARGET_DEG - 1.0
                          ? (direction < 0 ? "LEFT_180" : "RIGHT_180")
                          : (turnLeft ? "LEFT_90" : "RIGHT_90");
  printTurnStatus(label, speedCommand);
}

void postTurnForwardVerifyStep()
{
  readSensors();
  printSensors("POST_TURN");

  if (middleBandOnLine())
  {
    stopAllMotors();
    activeTurnDirection = 0;
    pendingTurnDirection = 0;
    ignoreRightAngleUntilMs = millis() + POST_TURN_IGNORE_RIGHT_ANGLE_MS;
    postTurnFineLineUntilMs = millis() + POST_TURN_FINE_LINE_MS;
    Serial.println("POST_TURN | Center line found during forward verify. Fine line tracking.");
    enterState(SENSE_LINE);
    return;
  }

  unsigned long elapsedMs = millis() - stateStartMs;
  float tiltDelta = fabs(calibratedTiltMagnitudeDeg() - postTurnStartTiltDeg);
  if (tiltDelta > postTurnMaxTiltDeltaDeg)
  {
    postTurnMaxTiltDeltaDeg = tiltDelta;
  }

  if (ultrasonicSensors[0].distanceCm > 0.0 && ultrasonicSensors[0].distanceCm <= POST_TURN_FRONT_BLOCK_CM)
  {
    startPostTurnRecovery("front_blocked");
    return;
  }

  if (elapsedMs >= POST_TURN_VERIFY_MS)
  {
    bool slopeVerificationActive = uphillTunnelPhaseReached() || returnTunnelActive;
    if (slopeVerificationActive &&
        postTurnMaxTiltDeltaDeg < POST_TURN_MIN_TILT_DELTA_DEG)
    {
      startPostTurnRecovery("no_slope_detected");
      return;
    }

    if (encoderMotionSince(postTurnStartEncoderCounts) < POST_TURN_MIN_ENCODER_TICKS)
    {
      startPostTurnRecovery("encoder_no_motion");
      return;
    }

    Serial.print(slopeVerificationActive
                     ? "POST_TURN | Slope transition verified. tiltDelta="
                     : "POST_TURN | Flat-field verify complete. tiltDelta=");
    Serial.println(postTurnMaxTiltDeltaDeg, 2);
    enterState(AFTER_TURN_SETTLE);
    return;
  }

  float lineError = 0.0;
  if (anySensorOnLine() && computeLinePidError(lineError))
  {
    driveLinePid(lineError);
  }
  else
  {
    driveForward(fieldNavigationActive() ? FIELD_POST_TURN_VERIFY_SPEED : POST_TURN_VERIFY_SPEED);
  }
}

void postTurnBackUpStep()
{
  if (millis() - stateStartMs >= POST_TURN_BACKUP_MS)
  {
    stopAllMotors();
    enterState(TURN_AROUND_180);
    return;
  }

  driveForward(fieldNavigationActive()
                   ? -max((int16_t)420, scaledSpeed(POST_TURN_BACKUP_SPEED, FIELD_TURN_SPEED_PERCENT))
                   : -POST_TURN_BACKUP_SPEED);
}

void enterFirstFindLineState()
{
  noLineTargetYawDeg = yawDeg;
  noLineYawTargetLocked = true;

  if (lastTurnDirection < 0)
  {
    findLineFirstDirection = -1;
    enterState(FIND_LINE_SMALL_LEFT);
  }
  else
  {
    findLineFirstDirection = 1;
    enterState(FIND_LINE_SMALL_RIGHT);
  }
}

void enterNoLineFindState()
{
  noLineTargetYawDeg = yawDeg;
  noLineYawTargetLocked = true;

  if (uphillTunnelPhaseReached())
  {
    Serial.print("NO_LINE | Uphill route active. Skip swing search, yaw=");
    Serial.println(noLineTargetYawDeg, 1);
    uphillLineFlickerYawDeg = noLineTargetYawDeg;
    enterUphillFlickerBypass("NO_LINE_UPHILL_ROUTE");
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  if (wallFollowingAllowed() && sideWallCloseForNoLineEntry())
  {
    Serial.print("NO_LINE | Side wall seen at line loss. Return yaw for tunnel, yaw=");
    Serial.println(noLineTargetYawDeg, 1);
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  findLineFirstDirection = lastDirection < 0 ? -1 : 1;

  Serial.print("NO_LINE | Search first. Hold original yaw ");
  Serial.print(noLineTargetYawDeg, 1);
  Serial.print(", first swing ");
  Serial.println(findLineFirstDirection < 0 ? "LEFT" : "RIGHT");

  enterState(findLineFirstDirection < 0 ? FIND_LINE_SMALL_LEFT : FIND_LINE_SMALL_RIGHT);
}

bool isFirstFindLineDirection(bool turnLeft)
{
  return (turnLeft && findLineFirstDirection < 0) ||
         (!turnLeft && findLineFirstDirection > 0);
}

uint16_t findLineSmallDuration(bool turnLeft)
{
  return isFirstFindLineDirection(turnLeft) ? FIND_LINE_SMALL_MS : FIND_LINE_SMALL_MS * 2;
}

uint16_t findLineBigDuration(bool turnLeft)
{
  return isFirstFindLineDirection(turnLeft) ? FIND_LINE_BIG_MS : FIND_LINE_BIG_MS * 2;
}

float findLineSmallSweepDeg(bool turnLeft)
{
  return isFirstFindLineDirection(turnLeft)
             ? FIND_LINE_SMALL_SWEEP_DEG
             : FIND_LINE_SMALL_SWEEP_DEG * 1.4;
}

float findLineBigSweepDeg(bool turnLeft)
{
  return isFirstFindLineDirection(turnLeft)
             ? FIND_LINE_BIG_SWEEP_DEG
             : FIND_LINE_BIG_SWEEP_DEG * 1.25;
}

RobotState nextSmallFindLineState(bool turnLeft)
{
  if (isFirstFindLineDirection(turnLeft))
  {
    return turnLeft ? FIND_LINE_SMALL_RIGHT : FIND_LINE_SMALL_LEFT;
  }

  return findLineFirstDirection < 0 ? FIND_LINE_BIG_LEFT : FIND_LINE_BIG_RIGHT;
}

RobotState nextBigFindLineState(bool turnLeft)
{
  if (isFirstFindLineDirection(turnLeft))
  {
    return turnLeft ? FIND_LINE_BIG_RIGHT : FIND_LINE_BIG_LEFT;
  }

  return NO_LINE_RETURN_YAW;
}

float normalizedYawErrorDeg(float currentDeg, float targetDeg)
{
  float error = currentDeg - targetDeg;
  while (error > 180.0)
  {
    error -= 360.0;
  }
  while (error < -180.0)
  {
    error += 360.0;
  }
  return error;
}

bool sideWallCloseForNoLineEntry()
{
  float leftCm = ultrasonicSensors[1].distanceCm;
  float rightCm = ultrasonicSensors[2].distanceCm;
  bool leftClose = leftCm > 0.0 && leftCm <= NO_LINE_WALL_ENTRY_SIDE_CM;
  bool rightClose = rightCm > 0.0 && rightCm <= NO_LINE_WALL_ENTRY_SIDE_CM;
  return leftClose || rightClose;
}

void findLineStep(bool turnLeft, uint16_t timeoutMs, float targetSweepDeg, RobotState nextState)
{
  if (centerOrAnySensorOnLine())
  {
    if (uphillLineIgnoreActive())
    {
      stopAllMotors();
      Serial.println("UPHILL_GUARD | Line seen during search, ignored. Returning yaw for climb.");
      enterState(NO_LINE_RETURN_YAW);
      return;
    }
    if (recordUphillLineFlicker(true, "FIND_FOUND"))
    {
      stopAllMotors();
      enterState(NO_LINE_RETURN_YAW);
      return;
    }
    stopAllMotors();
    ignoreRightAngleUntilMs = millis();
    noLineYawTargetLocked = false;
    enterState(SENSE_LINE);
    return;
  }

  bool bigFindState = state == FIND_LINE_BIG_LEFT || state == FIND_LINE_BIG_RIGHT;
  if (wallFollowingAllowed() &&
      bigFindState &&
      millis() - stateStartMs >= NO_LINE_WALL_ENTRY_MIN_MS &&
      sideWallCloseForNoLineEntry())
  {
    stopAllMotors();
    Serial.println("NO_LINE | Side wall detected during big search. Return yaw for wall follow.");
    enterState(NO_LINE_RETURN_YAW);
    return;
  }

  float sweptDeg = turnLeft
                       ? (findLineStartYawDeg - yawDeg)
                       : (yawDeg - findLineStartYawDeg);
  if (sweptDeg >= targetSweepDeg || millis() - stateStartMs >= timeoutMs)
  {
    stopAllMotors();
    enterState(nextState);
    return;
  }

  if (turnLeft)
  {
    pivotLeft(fieldNavigationActive() ? FIELD_FIND_LINE_SPEED : FIND_LINE_SPEED);
  }
  else
  {
    pivotRight(fieldNavigationActive() ? FIELD_FIND_LINE_SPEED : FIND_LINE_SPEED);
  }
}

void enterNoLineForwardMode()
{
  stopAllMotors();
  enterState(wallFollowingAllowed() ? WALL_FOLLOW : NO_LINE_IMU_FORWARD);
}

void noLineReturnYawStep()
{
  readSensors();
  printSensors("RETURN_YAW");

  bool lineSeen = anySensorOnLine();
  if (lineSeen)
  {
    if (uphillLineIgnoreActive())
    {
      // During the slope mouth bypass, line flicker is deliberately ignored.
    }
    else if (recordUphillLineFlicker(true, "RETURN_YAW_FOUND"))
    {
      stopAllMotors();
    }
    else
    {
      stopAllMotors();
      ignoreRightAngleUntilMs = millis();
      noLineYawTargetLocked = false;
      Serial.println("NO_LINE | Line found while returning yaw.");
      enterState(SENSE_LINE);
      return;
    }
  }

  float yawError = normalizedYawErrorDeg(yawDeg, noLineTargetYawDeg);
  if (fabs(yawError) <= NO_LINE_RETURN_YAW_TOLERANCE_DEG)
  {
    Serial.print("NO_LINE | Yaw restored error=");
    Serial.println(yawError, 1);
    enterNoLineForwardMode();
    return;
  }

  if (millis() - stateStartMs >= NO_LINE_RETURN_YAW_TIMEOUT_MS)
  {
    Serial.print("NO_LINE | Return yaw timeout error=");
    Serial.println(yawError, 1);
    enterNoLineForwardMode();
    return;
  }

  if (yawError > 0.0)
  {
    pivotLeft(fieldNavigationActive() ? FIELD_FIND_LINE_SPEED : NO_LINE_RETURN_YAW_SPEED);
  }
  else
  {
    pivotRight(fieldNavigationActive() ? FIELD_FIND_LINE_SPEED : NO_LINE_RETURN_YAW_SPEED);
  }
}

void noLineImuForwardStep()
{
  readSensors();
  printSensors("NO_LINE");

  if (anySensorOnLine())
  {
    if (uphillLineIgnoreActive())
    {
      // Keep climbing through the slope-mouth flicker instead of re-entering line follow.
    }
    else if (recordUphillLineFlicker(true, "NO_LINE_FORWARD_FOUND"))
    {
      stopAllMotors();
      enterState(NO_LINE_RETURN_YAW);
      return;
    }
    else
    {
      stopAllMotors();
      ignoreRightAngleUntilMs = millis();
      noLineYawTargetLocked = false;
      Serial.println("NO_LINE | Line detected, switching to line following.");
      enterState(SENSE_LINE);
      return;
    }
  }

  if (!USE_IMU)
  {
    driveForward(fieldNavigationActive() ? FIELD_NO_LINE_FORWARD_SPEED : NO_LINE_FORWARD_SPEED);
    return;
  }

  int16_t noLineForwardSpeed = fieldNavigationActive()
                                   ? FIELD_NO_LINE_FORWARD_SPEED
                                   : NO_LINE_FORWARD_SPEED;
  float yawError = normalizedYawErrorDeg(yawDeg, noLineTargetYawDeg);
  int16_t yawMaxCorrection = fieldNavigationActive()
                                 ? FIELD_NO_LINE_YAW_MAX_CORRECTION
                                 : NO_LINE_YAW_MAX_CORRECTION;
  int16_t correction = constrain(
      (int16_t)(yawError * NO_LINE_YAW_KP),
      -yawMaxCorrection,
      yawMaxCorrection);

  int16_t leftSpeed = constrain(
      (int16_t)(noLineForwardSpeed + correction),
      (int16_t)(noLineForwardSpeed - yawMaxCorrection),
      (int16_t)(noLineForwardSpeed + yawMaxCorrection));
  int16_t rightSpeed = constrain(
      (int16_t)(noLineForwardSpeed - correction),
      (int16_t)(noLineForwardSpeed - yawMaxCorrection),
      (int16_t)(noLineForwardSpeed + yawMaxCorrection));

  driveBySide(leftSpeed, rightSpeed);
}

bool validWallDistance(float distanceCm)
{
  return distanceCm > 0.0 && distanceCm <= WALL_SIDE_VALID_MAX_CM;
}

bool validFrontDistance(float distanceCm)
{
  return distanceCm > 0.0 && distanceCm < 150.0;
}

void wallFollowStep()
{
  readSensors();
  printSensors("WALL");

  if (returnTunnelActive && !anySensorOnLine())
  {
    returnTunnelSawNoLine = true;
  }

  bool stableFieldLineCandidate =
      millis() - stateStartMs >= WALL_LINE_REACQUIRE_MIN_MS &&
      (!returnTunnelActive || returnTunnelSawNoLine) &&
      sensorOnLine[SENSOR_MIDDLE];
  if (stableFieldLineCandidate)
  {
    if (wallLineReacquireSinceMs == 0)
    {
      wallLineReacquireSinceMs = millis();
      Serial.println(returnTunnelActive
                         ? "WALL | Center line candidate detected; confirming base-side line."
                         : "WALL | Center line candidate detected; confirming field entry.");
    }
    else if (millis() - wallLineReacquireSinceMs >= WALL_LINE_REACQUIRE_STABLE_MS)
    {
      stopAllMotors();
      uphillLineIgnoreUntilMs = 0;
      uphillLineFlickerHasLast = false;
      uphillLineFlickerTransitions = 0;
      uphillLineFlickerWindowStartMs = 0;
      airlockRouteTracking = false;
      airlockRouteTurns = 0;
      airlockDirectTunnelEntryArmed = false;
      noLineYawTargetLocked = false;
      ignoreRightAngleUntilMs = millis() + 250;
      resetLinePid();
      if (returnTunnelActive)
      {
        returnTunnelActive = false;
        returnTunnelSawNoLine = false;
        returnModeActive = false;
        returnPlanPending = false;
        returnPositionKnown = false;
        fieldModeActive = false;
        returnBaseReacquireSettleActive = true;
        ignoreRightAngleUntilMs = millis() + RETURN_BASE_IGNORE_TURN_MS;
        ignoreFrontObstacleUntilMs = millis() + RETURN_BASE_FINE_LINE_MS;
        postTurnFineLineUntilMs = millis() + RETURN_BASE_FINE_LINE_MS;
        Serial.println("RETURN | Tunnel complete. Base center line acquired; stop, settle, then lock onto PID line follow.");
        enterState(SENSOR_SETTLE);
        return;
      }
      else
      {
        fieldModeActive = true;
        Serial.println("WALL | Stable center line confirmed. Field entered; PID line following restored.");
      }
      enterState(SENSE_LINE);
      return;
    }
  }
  else
  {
    wallLineReacquireSinceMs = 0;
  }

  // Ignore edge hits and brief broken-line flicker. Wall-follow exits only
  // after the middle sensor confirms a stable line above.

  float leftCm = ultrasonicSensors[1].distanceCm;
  float rightCm = ultrasonicSensors[2].distanceCm;
  float frontCm = ultrasonicSensors[0].distanceCm;
  bool frontValid = validFrontDistance(frontCm);
  bool leftValid = validWallDistance(leftCm);
  bool rightValid = validWallDistance(rightCm);
  bool leftCenterValid = leftValid && leftCm <= WALL_CENTER_VALID_MAX_CM;
  bool rightCenterValid = rightValid && rightCm <= WALL_CENTER_VALID_MAX_CM;
  bool leftClose = leftCm > 0.0 && leftCm <= WALL_MIN_SIDE_CLEARANCE_CM;
  bool rightClose = rightCm > 0.0 && rightCm <= WALL_MIN_SIDE_CLEARANCE_CM;
  bool sideCritical = (leftCm > 0.0 && leftCm <= WALL_SIDE_CRITICAL_CM) ||
                      (rightCm > 0.0 && rightCm <= WALL_SIDE_CRITICAL_CM);

  bool uphillDoorHold = frontValid &&
                        frontCm <= WALL_FRONT_STOP_CM &&
                        !returnTunnelActive;
  if (frontValid && frontCm <= WALL_FRONT_STOP_CM && returnTunnelActive)
  {
    Serial.print("WALL | Front close ");
    printDistance(frontCm);
    Serial.println("cm. Start fixed backup/U-turn recovery.");
    startFixedRecovery("wall_front_close");
    return;
  }

  float error = 0.0; // Positive steers right, negative steers left.
  float kp = WALL_YAW_HOLD_KP;

  if (leftClose && rightClose)
  {
    error = rightCm - leftCm;
    kp = WALL_CLEARANCE_KP * 0.7;
  }
  else if (leftClose)
  {
    error = WALL_MIN_SIDE_CLEARANCE_CM - leftCm;
    kp = WALL_CLEARANCE_KP;
  }
  else if (rightClose)
  {
    error = -(WALL_MIN_SIDE_CLEARANCE_CM - rightCm);
    kp = WALL_CLEARANCE_KP;
  }
  else if (leftCenterValid && rightCenterValid)
  {
    // Center only when both side readings look like real tunnel walls.
    // Positive means the right wall is farther away, so steer right.
    error = rightCm - leftCm;
    kp = WALL_CENTER_KP;
  }
  else if (leftValid)
  {
    // Track a single visible left wall at WALL_TARGET_CM. Positive error steers right.
    error = WALL_TARGET_CM - leftCm;
    kp = WALL_CENTER_KP;
  }
  else if (rightValid)
  {
    // Track a single visible right wall at WALL_TARGET_CM. Positive error steers right.
    error = rightCm - WALL_TARGET_CM;
    kp = WALL_CENTER_KP;
  }
  else if (USE_IMU)
  {
    error = -normalizedYawErrorDeg(yawDeg, noLineTargetYawDeg);
    kp = WALL_YAW_HOLD_KP;
  }

  unsigned long now = millis();
  float dtSec = 0.02;
  if (wallPdHasLast && now > wallPdLastMs)
  {
    dtSec = (float)(now - wallPdLastMs) / 1000.0;
    if (dtSec < 0.005)
    {
      dtSec = 0.005;
    }
    else if (dtSec > 0.2)
    {
      dtSec = 0.2;
    }
  }

  float derivative = wallPdHasLast ? (error - wallPdLastError) / dtSec : 0.0;
  float correctionFloat = kp * error + WALL_PD_KD * derivative;
  int16_t correction = constrain(
      (int16_t)correctionFloat,
      (int16_t)-WALL_PD_MAX_CORRECTION,
      (int16_t) WALL_PD_MAX_CORRECTION);

  bool climbPowerActive = millis() - stateStartMs < WALL_CLIMB_POWER_MS;
  int16_t wallCruiseSpeed = climbPowerActive ? WALL_FORWARD_SPEED : WALL_FIELD_APPROACH_SPEED;
  int16_t wallMinimumSpeed = climbPowerActive ? WALL_MIN_DRIVE_SPEED : WALL_FIELD_APPROACH_MIN_SPEED;
  int16_t wallTargetSpeed = wallCruiseSpeed;
  if (frontValid && frontCm < WALL_FRONT_SLOW_CM)
  {
    float ratio = (frontCm - WALL_FRONT_STOP_CM) / (WALL_FRONT_SLOW_CM - WALL_FRONT_STOP_CM);
    ratio = constrain(ratio, 0.0, 1.0);
    int16_t approachMinimum = returnTunnelActive
                                  ? wallMinimumSpeed
                                  : WALL_UPHILL_DOOR_HOLD_SPEED;
    wallTargetSpeed = (int16_t)(approachMinimum +
                                ratio * (wallCruiseSpeed - approachMinimum));
  }
  if (uphillDoorHold)
  {
    wallTargetSpeed = WALL_UPHILL_DOOR_HOLD_SPEED;
  }
  if (sideCritical && wallTargetSpeed > wallMinimumSpeed)
  {
    wallTargetSpeed = wallMinimumSpeed;
  }

  float rampDtSec = wallSpeedRampLastMs == 0
                        ? 0.02
                        : (float)(now - wallSpeedRampLastMs) / 1000.0;
  rampDtSec = constrain(rampDtSec, 0.005, 0.2);
  float maxSpeedChange = (wallTargetSpeed < wallAppliedBaseSpeed
                              ? WALL_SPEED_DECEL_PER_SEC
                              : WALL_SPEED_ACCEL_PER_SEC) *
                         rampDtSec;
  if (fabs((float)wallTargetSpeed - wallAppliedBaseSpeed) <= maxSpeedChange)
  {
    wallAppliedBaseSpeed = wallTargetSpeed;
  }
  else if (wallTargetSpeed < wallAppliedBaseSpeed)
  {
    wallAppliedBaseSpeed -= maxSpeedChange;
  }
  else
  {
    wallAppliedBaseSpeed += maxSpeedChange;
  }
  wallSpeedRampLastMs = now;
  int16_t wallBaseSpeed = (int16_t)wallAppliedBaseSpeed;
  int16_t wheelMinimumSpeed = uphillDoorHold || (!returnTunnelActive && frontValid && frontCm < WALL_FRONT_SLOW_CM)
                                  ? max((int16_t)160, (int16_t)(wallBaseSpeed - 100))
                                  : wallMinimumSpeed;

  int16_t leftSpeed = constrain(
      (int16_t)(wallBaseSpeed + correction),
      wheelMinimumSpeed,
      (int16_t)MAX_SPEED);
  int16_t rightSpeed = constrain(
      (int16_t)(wallBaseSpeed - correction),
      wheelMinimumSpeed,
      (int16_t)MAX_SPEED);

  driveBySide(leftSpeed, rightSpeed);

  if (now - wallFollowLastDebugMs >= WALL_CONTROL_DEBUG_INTERVAL_MS)
  {
    long currentCounts[ENCODER_COUNT];
    copyEncoderCounts(currentCounts);
    Serial.print("WALL_CTL | L=");
    printDistance(leftCm);
    Serial.print(" R=");
    printDistance(rightCm);
    Serial.print(" F=");
    printDistance(frontCm);
    Serial.print(" err=");
    Serial.print(error, 2);
    Serial.print(" kp=");
    Serial.print(kp, 1);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" base=");
    Serial.print(wallBaseSpeed);
    Serial.print(" target=");
    Serial.print(wallTargetSpeed);
    Serial.print(" doorHold=");
    Serial.print(uphillDoorHold ? "Y" : "N");
    Serial.print(" climbPower=");
    Serial.print(climbPowerActive ? "Y" : "N");
    Serial.print(" critical=");
    Serial.print(sideCritical ? "Y" : "N");
    Serial.print(" speedL=");
    Serial.print(leftSpeed);
    Serial.print(" speedR=");
    Serial.print(rightSpeed);
    Serial.print(" | dEnc RF=");
    Serial.print(currentCounts[0] - wallFollowLastEncoderCounts[0]);
    Serial.print(" RR=");
    Serial.print(currentCounts[1] - wallFollowLastEncoderCounts[1]);
    Serial.print(" LR=");
    Serial.print(currentCounts[2] - wallFollowLastEncoderCounts[2]);
    Serial.print(" LF=");
    Serial.print(currentCounts[3] - wallFollowLastEncoderCounts[3]);
    Serial.print(" | dPitch=");
    Serial.print(calibratedPitchDeltaDeg(), 1);
    Serial.print(" dRoll=");
    Serial.print(calibratedRollDeltaDeg(), 1);
    Serial.print(" dTilt=");
    Serial.println(calibratedTiltMagnitudeDeg(), 1);
    copyEncoderCounts(wallFollowLastEncoderCounts);
    wallFollowLastDebugMs = now;
  }

  wallPdLastError = error;
  wallPdLastMs = now;
  wallPdHasLast = true;
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
  if (!USE_SEED_PLANTER)
  {
    return;
  }

  planterServo.attach(SERVO_SIGNAL_PIN);
  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  planterState = PLANTER_IDLE;

  Serial.print("PLANTER | Servo enabled on D");
  Serial.print(SERVO_SIGNAL_PIN);
  Serial.print(", sweep ");
  Serial.print(PLANTER_MIN_ANGLE);
  Serial.print("..");
  Serial.print(PLANTER_MAX_ANGLE);
  Serial.println(" deg.");
}

bool triggerSeedPlanter(const char *source)
{
  if (!USE_SEED_PLANTER)
  {
    return false;
  }

  if (planterState != PLANTER_IDLE)
  {
    Serial.print("PLANTER | Busy, trigger ignored from ");
    Serial.print(source);
    Serial.print(". state=");
    Serial.println(planterStateName());
    return false;
  }

  if (lastPlantTriggerMs != 0 && millis() - lastPlantTriggerMs < RFID_PLANT_COOLDOWN_MS)
  {
    Serial.print("PLANTER | Cooldown, trigger ignored from ");
    Serial.println(source);
    return false;
  }

  lastPlantTriggerMs = millis();
  runRandomActionAfterPlant = true;
  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);
  planterLastStepMs = millis();
  planterState = PLANTER_SWEEP_FORWARD;

  Serial.print("PLANTER | Triggered by ");
  Serial.println(source);
  return true;
}

void updateSeedPlanter()
{
  if (!USE_SEED_PLANTER || planterState == PLANTER_IDLE)
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
        Serial.println("PLANTER | Done");
        if (plantingSeedReportArmed)
        {
          plantingSeedReportArmed = false;
          copyText(pendingSeedReportTagId, sizeof(pendingSeedReportTagId), plantingSeedTagId);
          pendingSeedReport = pendingSeedReportTagId[0] != '\0';
          if (pendingSeedReport)
          {
            Serial.print("SERVER | Queued seedPlanted for tag_id=");
            Serial.println(pendingSeedReportTagId);
          }
        }
        if (runRandomActionAfterPlant)
        {
          runRandomActionAfterPlant = false;
          startPostPlantRandomAction();
        }
      }
      break;

    case PLANTER_IDLE:
      break;
  }
}

void startPostPlantRandomAction()
{
  stopAllMotors();
  readSensors();
  printSensorsNow("POST_PLANT_DECIDE");

  bool lineSeen = anySensorOnLine();
  bool leftBranchSeen =
      sensorValues[SENSOR_SLIGHT_LEFT] >= T_JUNCTION_SIDE_THRESHOLD &&
      sensorValues[SENSOR_LEFT] >= T_JUNCTION_SIDE_THRESHOLD;
  bool rightBranchSeen =
      sensorValues[SENSOR_RIGHT] >= T_JUNCTION_SIDE_THRESHOLD &&
      sensorValues[SENSOR_SLIGHT_RIGHT] >= T_JUNCTION_SIDE_THRESHOLD;

  if (!lineSeen)
  {
    Serial.println("PLANTER | No line after seed. Search slowly before choosing a route.");
    enterNoLineFindState();
    return;
  }

  // Turns are allowed only when the reflectance array can prove that branch
  // exists. At a real junction, bias toward turning to improve exploration.
  int action = random(0, 4);
  if (leftBranchSeen && rightBranchSeen)
  {
    action = random(0, 2) == 0 ? 1 : 2;
  }
  else if (leftBranchSeen && action != 0)
  {
    action = 1;
  }
  else if (rightBranchSeen && action != 0)
  {
    action = 2;
  }
  else
  {
    action = 0;
  }

  if (action == 0)
  {
    Serial.println("PLANTER | Safe action after seed: FOLLOW_LINE");
    ignoreRightAngleUntilMs = millis() + 600;
    enterState(SENSE_LINE);
    return;
  }

  if (action == 1)
  {
    Serial.println("PLANTER | Confirmed left branch after seed: LEFT_90");
    pendingTurnDirection = -1;
    enterState(TURN_LEFT_90);
    return;
  }

  Serial.println("PLANTER | Confirmed right branch after seed: RIGHT_90");
  pendingTurnDirection = 1;
  enterState(TURN_RIGHT_90);
}

bool initRFIDNoResetPin()
{
  Wire1.beginTransmission(RFID_I2C_ADDR);
  if (Wire1.endTransmission() != 0)
  {
    Serial.println("RFID | Not found at I2C address 0x28.");
    return false;
  }

  // Same core setup as MFRC522_I2C::PCD_Init(), but without touching reset pin.
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
  Serial.print("RFID | Ready at 0x28, version register=0x");
  if (version < 0x10)
  {
    Serial.print("0");
  }
  Serial.println(version, HEX);
  Serial.print("RFID | Antenna gain=0x");
  Serial.println(mfrc522.PCD_GetAntennaGain(), HEX);

  if (version == 0x00 || version == 0xFF)
  {
    Serial.println("RFID | Version read failed. Check SDA1/SCL1, VCC, GND, and I2C address.");
    return false;
  }

  return true;
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

bool readRFIDCard(uint32_t &uidHash, bool printCard)
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
    if (RFID_VERBOSE_DEBUG && millis() - lastRFIDDebugMs >= RFID_DEBUG_INTERVAL_MS)
    {
      lastRFIDDebugMs = millis();
      Serial.print("RFID | No card response, status=");
      Serial.println(status);
    }
    return false;
  }

  if (RFID_VERBOSE_DEBUG)
  {
    Serial.print("RFID | Card response, status=");
    Serial.print(status);
    Serial.print(" ATQA=");
    for (byte i = 0; i < atqaSize && i < sizeof(atqa); i++)
    {
      Serial.print(atqa[i] < 0x10 ? " 0" : " ");
      Serial.print(atqa[i], HEX);
    }
    Serial.println();
  }

  if (!mfrc522.PICC_ReadCardSerial())
  {
    if (RFID_VERBOSE_DEBUG)
    {
      Serial.println("RFID | Card present but UID read failed.");
    }
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
  lastRFIDUidHash = uidHash;
  haveLastRFIDUid = true;
  lastRFIDUidMs = millis();

  if (printCard)
  {
    Serial.print("RFID | Card ID:");
    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();
  }

  mfrc522.PICC_HaltA();
  return true;
}

void planEmergencyReturn()
{
  returnPlanPending = false;
  stopAllMotors();
  if (!returnPositionKnown)
  {
    Serial.println("RETURN | Position unknown. Move cautiously and query next RFID.");
    ignoreRightAngleUntilMs = millis() + 1200;
    enterState(SENSE_LINE);
    return;
  }

  if (returnGridX == RETURN_EXIT_X && returnGridY == RETURN_EXIT_Y)
  {
    Serial.println("RETURN | Exit (9,7) reached. Enter tunnel wall-follow.");
    returnTunnelActive = true;
    returnTunnelSawNoLine = false;
    fieldModeActive = false;
    airlockRouteTracking = true;
    airlockRouteTurns = AIRLOCK_UPHILL_ROUTE_TURNS;
    noLineTargetYawDeg = yawDeg;
    noLineYawTargetLocked = true;
    enterState(WALL_FOLLOW);
    return;
  }

  int8_t desiredHeading = gridHeading;
  if (returnGridY < RETURN_EXIT_Y)
  {
    desiredHeading = 0;
  }
  else if (returnGridY > RETURN_EXIT_Y)
  {
    desiredHeading = 2;
  }
  else if (returnGridX < RETURN_EXIT_X)
  {
    desiredHeading = 1;
  }
  else if (returnGridX > RETURN_EXIT_X)
  {
    desiredHeading = 3;
  }

  uint8_t delta = (desiredHeading - gridHeading + 4) % 4;
  Serial.print("RETURN | plan pos=");
  Serial.print(returnGridX);
  Serial.print(",");
  Serial.print(returnGridY);
  Serial.print(" heading=");
  Serial.print(gridHeading);
  Serial.print(" desired=");
  Serial.println(desiredHeading);

  if (delta == 0)
  {
    noLineTargetYawDeg = yawDeg;
    noLineYawTargetLocked = true;
    ignoreRightAngleUntilMs = millis() + 1800;
    enterState(SENSE_LINE);
  }
  else if (delta == 1)
  {
    enterState(TURN_RIGHT_90);
  }
  else if (delta == 3)
  {
    enterState(TURN_LEFT_90);
  }
  else
  {
    enterState(TURN_AROUND_180);
  }
}

void startRFIDPause()
{
  bool firstCard = !rfidFirstCardSeen;
  rfidFirstCardSeen = true;
  if (firstCard && !returnModeActive)
  {
    startAirlockUphillRouteTracking(USE_SERVER_MESSENGER ? "FIRST_RFID_AIRLOCK" : "FIRST_RFID_LOCAL");
  }
  rfidPauseShouldPlant = !firstCard;
  rfidReturnState = state == RFID_PAUSE ? SENSE_LINE : state;
  lastRFIDActionMs = millis();
  lastRFIDActionUidHash = lastRFIDUidHash;
  haveLastRFIDActionUid = haveLastRFIDUid;
  copyText(activeRFIDTagId, sizeof(activeRFIDTagId), lastRFIDTagId);
  serverRfidRequest = SERVER_RFID_NONE;
  serverRfidReplyReceived = false;
  serverFertile = false;
  serverPlanted = false;
  serverAirlockAccepted = false;
  serverShouldReportCurrentPlant = false;

  Serial.print("RFID | tag_id=");
  Serial.print(activeRFIDTagId[0] == '\0' ? "UNKNOWN" : activeRFIDTagId);

  bool serverReady = USE_SERVER_MESSENGER && serverMessengerStarted && serverMessenger.isConnected();
  if (serverReady)
  {
    if (firstCard && !returnModeActive)
    {
      rfidPauseShouldPlant = false;
      serverRfidRequest = SERVER_RFID_WAIT_AIRLOCK;
      serverRfidRequestMs = millis();
      Serial.println(" first card: open airlock, no seed.");
      if (!sendServerOpenAirlock(activeRFIDTagId))
      {
        serverRfidRequest = SERVER_RFID_NONE;
      }
    }
    else
    {
      rfidPauseShouldPlant = false;
      serverRfidRequest = SERVER_RFID_WAIT_FERTILITY;
      serverRfidRequestMs = millis();
      Serial.println(" query fertile before planting.");
      if (!sendServerIsFertile(activeRFIDTagId))
      {
        serverRfidRequest = SERVER_RFID_NONE;
        Serial.println("RFID | Server query failed. No seed without fertile reply.");
      }
    }
  }
  else if (ALLOW_LOCAL_RFID_FALLBACK)
  {
    Serial.print(" local fallback: ");
    Serial.println(rfidPauseShouldPlant ? "pause then plant." : "first card pause only.");
  }
  else
  {
    rfidPauseShouldPlant = false;
    Serial.println(" online mode: server not connected. Pause only, no seed.");
  }

  enterState(RFID_PAUSE);
}

void handleRFIDPauseState()
{
  stopAllMotors();

  bool waitingForServer = serverRfidRequest == SERVER_RFID_WAIT_FERTILITY && !serverRfidReplyReceived;
  bool serverTimedOut = waitingForServer && millis() - serverRfidRequestMs >= SERVER_REPLY_TIMEOUT_MS;

  if (waitingForServer && !serverTimedOut)
  {
    return;
  }

  if (serverTimedOut)
  {
    if (serverRfidRequest == SERVER_RFID_WAIT_FERTILITY)
    {
      rfidPauseShouldPlant = false;
      serverShouldReportCurrentPlant = false;
      Serial.println("SERVER | Fertility reply timeout. No seed.");
    }
    serverRfidReplyReceived = true;
  }

  if (millis() - stateStartMs < RFID_PAUSE_MS)
  {
    return;
  }

  if (returnModeActive)
  {
    rfidPauseShouldPlant = false;
    serverRfidRequest = SERVER_RFID_NONE;
    serverShouldReportCurrentPlant = false;
    returnPlanPending = true;
    Serial.println("RETURN | RFID pause complete; skip planting and replan.");
    enterState(SENSE_LINE);
    return;
  }

  if (rfidPauseShouldPlant)
  {
    bool started = triggerSeedPlanter(serverShouldReportCurrentPlant ? "RFID_SERVER" : "RFID_LOCAL");
    if (started && serverShouldReportCurrentPlant)
    {
      copyText(plantingSeedTagId, sizeof(plantingSeedTagId), activeRFIDTagId);
      plantingSeedReportArmed = plantingSeedTagId[0] != '\0';
    }
  }
  else
  {
    if (serverRfidRequest == SERVER_RFID_WAIT_FERTILITY)
    {
      Serial.print("RFID | No planting. fertile=");
      Serial.print(serverFertile ? "true" : "false");
      Serial.print(" planted=");
      Serial.println(serverPlanted ? "true" : "false");
    }
    else
    {
      Serial.println("RFID | First-card pause complete.");
    }
  }

  rfidPauseShouldPlant = false;
  serverRfidRequest = SERVER_RFID_NONE;
  serverShouldReportCurrentPlant = false;
  enterState(rfidReturnState == RFID_PAUSE ? SENSE_LINE : rfidReturnState);
}

void updateRFID()
{
  if (!USE_RFID)
  {
    return;
  }

  if (!rfidReady)
  {
    if (millis() - lastRFIDRetryMs >= RFID_RETRY_INTERVAL_MS)
    {
      lastRFIDRetryMs = millis();
      rfidReady = initRFIDNoResetPin();
    }
    return;
  }

  if (state == GRID_OBSTACLE_AVOID)
  {
    return;
  }

  // A tag remains underneath the reader while pivoting on a node. Pausing a
  // turn here resets its IMU target and can trap the robot spinning over the
  // same tag instead of completing the turn and reacquiring the line.
  switch (state)
  {
    case PRE_TURN_FORWARD_LEFT:
    case PRE_TURN_FORWARD_RIGHT:
    case TURN_LEFT_90:
    case TURN_RIGHT_90:
    case TURN_AROUND_180:
    case POST_TURN_FORWARD_VERIFY:
    case POST_TURN_BACK_UP:
    case AFTER_TURN_SETTLE:
      return;

    default:
      break;
  }

  if (USE_GRID_OBSTACLE_AVOIDANCE &&
      gridObstacleAvoidanceAllowed() &&
      gridObstacleDetectedAhead(ultrasonicSensors[0].distanceCm))
  {
    return;
  }

  if (state == RFID_PAUSE ||
      (lastRFIDActionMs != 0 && millis() - lastRFIDActionMs < RFID_CARD_ACTION_COOLDOWN_MS))
  {
    return;
  }

  if (millis() - lastRFIDCheckMs < RFID_CHECK_INTERVAL_MS)
  {
    return;
  }
  lastRFIDCheckMs = millis();

  uint32_t uidHash = 0;
  if (readRFIDCard(uidHash, false))
  {
    if (haveLastRFIDActionUid &&
        uidHash == lastRFIDActionUidHash &&
        millis() - lastRFIDActionMs < RFID_SAME_CARD_ACTION_COOLDOWN_MS)
    {
      return;
    }

    startRFIDPause();
  }
}

const char *gridObstaclePhaseName(GridObstaclePhase phase)
{
  switch (phase)
  {
    case GRID_OBS_IDLE: return "IDLE";
    case GRID_OBS_TURN_OUT: return "TURN_OUT";
    case GRID_OBS_SIDE_STEP_OUT: return "SIDE_STEP_OUT";
    case GRID_OBS_TURN_FORWARD: return "TURN_FORWARD";
    case GRID_OBS_PASS_OBSTACLE: return "PASS_OBSTACLE";
    case GRID_OBS_TURN_BACK: return "TURN_BACK";
    case GRID_OBS_SIDE_STEP_BACK: return "SIDE_STEP_BACK";
    case GRID_OBS_TURN_ORIGINAL: return "TURN_ORIGINAL";
    case GRID_OBS_FORWARD_CLEAR: return "FORWARD_CLEAR";
    case GRID_OBS_DONE: return "DONE";
  }
  return "UNKNOWN";
}

void printGridObstaclePhase()
{
  Serial.print("GRID_OBS | phase=");
  Serial.print(gridObstaclePhaseName(gridObstaclePhase));
  Serial.print(" side=");
  Serial.print(gridObstacleSide < 0 ? "LEFT" : "RIGHT");
  Serial.print(" node=");
  Serial.print(gridObstacleNodeCount);
  Serial.print("/");
  Serial.print(gridObstacleNodeTarget);
  Serial.print(" yaw=");
  Serial.println(yawDeg, 1);
}

void setGridObstaclePhase(GridObstaclePhase phase)
{
  gridObstaclePhase = phase;
  gridObstaclePhaseStartMs = millis();
  gridObstacleTurnSettleStartMs = 0;
  printGridObstaclePhase();
}

void beginGridObstacleTurn(GridObstaclePhase phase, int direction)
{
  gridObstacleTurnDirection = direction < 0 ? -1 : 1;
  gridObstacleTurnStartYawDeg = yawDeg;
  resetTurnEncoderBalance();
  setGridObstaclePhase(phase);
}

void beginGridObstacleStraight(GridObstaclePhase phase, uint8_t targetNodes)
{
  gridObstacleNodeCount = 0;
  gridObstacleNodeTarget = targetNodes;
  if (haveLastRFIDUid)
  {
    gridObstacleLastNodeUid = lastRFIDUidHash;
    gridObstacleHasLastNode = true;
    gridObstacleLastNodeMs = millis();
  }
  setGridObstaclePhase(phase);
}

void finishGridObstacleAvoidance()
{
  stopAllMotors();
  gridObstaclePhase = GRID_OBS_IDLE;
  gridObstacleLastCompleteMs = millis();
  ignoreRightAngleUntilMs = millis() + 600;
  Serial.println("GRID_OBS | Done. Returned to original heading, resume line following.");
  enterState(SENSE_LINE);
}

void advanceGridObstacleAfterTurn()
{
  switch (gridObstaclePhase)
  {
    case GRID_OBS_TURN_OUT:
      beginGridObstacleStraight(GRID_OBS_SIDE_STEP_OUT, 1);
      break;

    case GRID_OBS_TURN_FORWARD:
      beginGridObstacleStraight(GRID_OBS_PASS_OBSTACLE, 3);
      break;

    case GRID_OBS_TURN_BACK:
      beginGridObstacleStraight(GRID_OBS_SIDE_STEP_BACK, 1);
      break;

    case GRID_OBS_TURN_ORIGINAL:
      beginGridObstacleStraight(GRID_OBS_FORWARD_CLEAR, 3);
      break;

    default:
      finishGridObstacleAvoidance();
      break;
  }
}

void advanceGridObstacleAfterStraight()
{
  switch (gridObstaclePhase)
  {
    case GRID_OBS_SIDE_STEP_OUT:
      beginGridObstacleTurn(GRID_OBS_TURN_FORWARD, -gridObstacleSide);
      break;

    case GRID_OBS_PASS_OBSTACLE:
      beginGridObstacleTurn(GRID_OBS_TURN_BACK, -gridObstacleSide);
      break;

    case GRID_OBS_SIDE_STEP_BACK:
      beginGridObstacleTurn(GRID_OBS_TURN_ORIGINAL, gridObstacleSide);
      break;

    case GRID_OBS_FORWARD_CLEAR:
      finishGridObstacleAvoidance();
      break;

    default:
      finishGridObstacleAvoidance();
      break;
  }
}

void startGridObstacleAvoidance()
{
  stopAllMotors();
  frontObstacleHolding = false;
  float leftCm = ultrasonicSensors[1].distanceCm;
  float rightCm = ultrasonicSensors[2].distanceCm;
  bool leftValid = leftCm > 0.0 && leftCm < 200.0;
  bool rightValid = rightCm > 0.0 && rightCm < 200.0;

  if (leftValid && rightValid)
  {
    gridObstacleSide = rightCm >= leftCm ? 1 : -1;
  }
  else if (rightValid)
  {
    gridObstacleSide = 1;
  }
  else if (leftValid)
  {
    gridObstacleSide = -1;
  }
  else
  {
    gridObstacleSide = 1;
  }

  gridObstacleHasLastNode = haveLastRFIDUid;
  gridObstacleLastNodeUid = lastRFIDUidHash;
  gridObstacleLastNodeMs = millis();

  Serial.print("GRID_OBS | Obstacle detected front=");
  printDistance(ultrasonicSensors[0].distanceCm);
  Serial.print("cm left=");
  printDistance(leftCm);
  Serial.print("cm right=");
  printDistance(rightCm);
  Serial.print("cm choose=");
  Serial.println(gridObstacleSide < 0 ? "LEFT" : "RIGHT");

  enterState(GRID_OBSTACLE_AVOID);
  beginGridObstacleTurn(GRID_OBS_TURN_OUT, gridObstacleSide);
}

bool gridObstacleReadNode()
{
  if (!rfidReady || millis() - gridObstaclePhaseStartMs < GRID_OBSTACLE_NODE_ARM_DELAY_MS)
  {
    return false;
  }

  if (millis() - lastRFIDCheckMs < RFID_CHECK_INTERVAL_MS)
  {
    return false;
  }
  lastRFIDCheckMs = millis();

  uint32_t uidHash = 0;
  if (!readRFIDCard(uidHash, false))
  {
    return false;
  }

  if (gridObstacleHasLastNode &&
      uidHash == gridObstacleLastNodeUid &&
      millis() - gridObstacleLastNodeMs < GRID_OBSTACLE_NODE_REPEAT_MS)
  {
    return false;
  }

  gridObstacleHasLastNode = true;
  gridObstacleLastNodeUid = uidHash;
  gridObstacleLastNodeMs = millis();
  gridObstacleNodeCount++;

  Serial.print("GRID_OBS | Node ");
  Serial.print(gridObstacleNodeCount);
  Serial.print("/");
  Serial.print(gridObstacleNodeTarget);
  Serial.print(" uid=0x");
  Serial.println(uidHash, HEX);
  return true;
}

void gridObstacleDriveStraight()
{
  readSensors();
  printSensors("GRID_OBS_LINE");

  float error = 0.0;
  if (anySensorOnLine() && computeLinePidError(error))
  {
    int16_t correction = constrain(
        (int16_t)(error * GRID_OBSTACLE_LINE_KP),
        (int16_t)-GRID_OBSTACLE_LINE_MAX_CORRECTION,
        (int16_t) GRID_OBSTACLE_LINE_MAX_CORRECTION);
    int16_t leftSpeed = constrain(
        (int16_t)(GRID_OBSTACLE_FORWARD_SPEED - correction),
        (int16_t)-MAX_SPEED,
        (int16_t) MAX_SPEED);
    int16_t rightSpeed = constrain(
        (int16_t)(GRID_OBSTACLE_FORWARD_SPEED + correction),
        (int16_t)-MAX_SPEED,
        (int16_t) MAX_SPEED);
    driveBySide(leftSpeed, rightSpeed);
  }
  else
  {
    driveForward(GRID_OBSTACLE_FORWARD_SPEED);
  }

  gridObstacleReadNode();
  if (gridObstacleNodeCount >= gridObstacleNodeTarget)
  {
    stopAllMotors();
    advanceGridObstacleAfterStraight();
    return;
  }

  if (millis() - gridObstaclePhaseStartMs >= GRID_OBSTACLE_STRAIGHT_TIMEOUT_MS)
  {
    Serial.println("GRID_OBS | Straight node timeout.");
    stopAllMotors();
    enterState(LINE_LOST_STOP);
  }
}

void gridObstacleTurnStep()
{
  float turnedDeg = fabs(yawDeg - gridObstacleTurnStartYawDeg);
  float error = GRID_OBSTACLE_TURN_TARGET_DEG - turnedDeg;

  if (error <= GRID_OBSTACLE_TURN_TOLERANCE_DEG)
  {
    stopAllMotors();
    if (gridObstacleTurnSettleStartMs == 0)
    {
      gridObstacleTurnSettleStartMs = millis();
      return;
    }

    if (millis() - gridObstacleTurnSettleStartMs >= GRID_OBSTACLE_TURN_SETTLE_MS)
    {
      advanceGridObstacleAfterTurn();
    }
    return;
  }

  if (millis() - gridObstaclePhaseStartMs >= GRID_OBSTACLE_TURN_TIMEOUT_MS)
  {
    Serial.print("GRID_OBS | Turn timeout phase=");
    Serial.print(gridObstaclePhaseName(gridObstaclePhase));
    Serial.print(" turned=");
    Serial.println(turnedDeg, 1);
    stopAllMotors();
    enterState(LINE_LOST_STOP);
    return;
  }

  int16_t speed = error > GRID_OBSTACLE_TURN_FINE_REMAINING_DEG
                      ? GRID_OBSTACLE_TURN_FAST_SPEED
                      : GRID_OBSTACLE_TURN_FINE_SPEED;
  driveTurnDirection(gridObstacleTurnDirection, speed);

  if (millis() - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS)
  {
    lastPrintMs = millis();
    Serial.print("GRID_OBS_TURN | phase=");
    Serial.print(gridObstaclePhaseName(gridObstaclePhase));
    Serial.print(" turned=");
    Serial.print(turnedDeg, 1);
    Serial.print("/90.0 error=");
    Serial.print(error, 1);
    Serial.print(" speed=");
    Serial.println(speed);
  }
}

void gridObstacleAvoidStep()
{
  switch (gridObstaclePhase)
  {
    case GRID_OBS_TURN_OUT:
    case GRID_OBS_TURN_FORWARD:
    case GRID_OBS_TURN_BACK:
    case GRID_OBS_TURN_ORIGINAL:
      gridObstacleTurnStep();
      break;

    case GRID_OBS_SIDE_STEP_OUT:
    case GRID_OBS_PASS_OBSTACLE:
    case GRID_OBS_SIDE_STEP_BACK:
    case GRID_OBS_FORWARD_CLEAR:
      gridObstacleDriveStraight();
      break;

    case GRID_OBS_DONE:
      finishGridObstacleAvoidance();
      break;

    case GRID_OBS_IDLE:
    default:
      finishGridObstacleAvoidance();
      break;
  }
}

void scanRFIDOnlyI2CBus()
{
  Serial.println("RFID_TEST | Scanning Wire1...");
  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0)
    {
      Serial.print("RFID_TEST | I2C found 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0)
  {
    Serial.println("RFID_TEST | No I2C devices found on Wire1.");
  }
}

void setupRFIDOnlyTest()
{
  Serial.begin(115200);
  delay(1000);
  setupSeedPlanter();

  Wire1.begin();
  Wire1.setClock(400000);
  scanRFIDOnlyI2CBus();

  rfidReady = initRFIDNoResetPin();
  Serial.println(rfidReady ? "RFID_TEST | Ready. Tap a card." : "RFID_TEST | Init failed, retrying in loop.");
}

void loopRFIDOnlyTest()
{
  updateRFID();
  updateSeedPlanter();
  delay(10);
}

void setupServoTest()
{
  Serial.begin(115200);
  delay(1000);

  planterServo.attach(SERVO_SIGNAL_PIN);
  planterAngle = PLANTER_MIN_ANGLE;
  planterServo.write(planterAngle);

  Serial.println("SERVO_TEST | Enabled. Main robot logic is disabled.");
  Serial.print("SERVO_TEST | Signal pin D");
  Serial.print(SERVO_SIGNAL_PIN);
  Serial.println(". Connect servo VCC to 5V, GND to GND/common ground.");
  Serial.println("SERVO_TEST | Pattern: sweep 0 -> 135 -> 0 degrees.");
}

void loopServoTest()
{
  static int direction = 1;
  static unsigned long lastStepMs = 0;
  static unsigned long pauseStartMs = 0;
  static bool paused = false;

  unsigned long nowMs = millis();

  if (paused)
  {
    if (nowMs - pauseStartMs >= PLANTER_END_PAUSE_MS)
    {
      paused = false;
      lastStepMs = nowMs;
    }
    return;
  }

  if (nowMs - lastStepMs < PLANTER_STEP_DELAY_MS)
  {
    return;
  }

  lastStepMs = nowMs;
  planterAngle += direction;

  if (planterAngle >= PLANTER_MAX_ANGLE)
  {
    planterAngle = PLANTER_MAX_ANGLE;
    direction = -1;
    paused = true;
    pauseStartMs = nowMs;
    Serial.println("SERVO_TEST | angle=135");
  }
  else if (planterAngle <= PLANTER_MIN_ANGLE)
  {
    planterAngle = PLANTER_MIN_ANGLE;
    direction = 1;
    paused = true;
    pauseStartMs = nowMs;
    Serial.println("SERVO_TEST | angle=0");
  }

  planterServo.write(planterAngle);
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

const char *lfTestPhaseName(LfTestPhase phase)
{
  switch (phase)
  {
    case LF_TEST_STOP_BEFORE: return "STOP_BEFORE";
    case LF_TEST_FORWARD: return "FORWARD";
    case LF_TEST_STOP_MID: return "STOP_MID";
    case LF_TEST_REVERSE: return "REVERSE";
    case LF_TEST_STOP_AFTER: return "STOP_AFTER";
  }
  return "UNKNOWN";
}

int16_t lfTestPhysicalCommand()
{
  switch (lfTestPhase)
  {
    case LF_TEST_FORWARD: return LF_TEST_SPEED;
    case LF_TEST_REVERSE: return -LF_TEST_SPEED;
    default: return 0;
  }
}

void setLFTestPhase(LfTestPhase nextPhase)
{
  lfTestPhase = nextPhase;
  lfTestPhaseStartMs = millis();
  lfTestLastPrintMs = 0;
  copyEncoderCounts(lfTestLastEncoderCounts);

  Serial.print("LF_TEST_PHASE | ");
  Serial.println(lfTestPhaseName(lfTestPhase));
}

void setupLFMotorEncoderTest()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  beginButton(reviveButton);
  beginButton(killButton);
  beginEncoders();

  Wire1.begin();
  Wire1.setClock(400000);
  scanI2CBus(Wire1, "Wire1 for LF motor test");

  setupMotoron(mcLower, RF, RR, LR);
  setupMotoron(mcUpper, LF);
  stopAllMotors();

  Serial.println("LF_TEST | Only LF motor should move. Lift the robot before running.");
  Serial.println("LF_TEST | Encoder order RF RR LR LF. LF encoder should be D8/D9.");
  Serial.println("LF_TEST | cmdPhysical + means LF physical forward; rawLF includes FWD_LF polarity.");
  Serial.println("LF_TEST | D45 toggles STOP/RUN.");

  if (killButton.stableState == LOW)
  {
    setKillStopped(true, "Mechanical boot D45");
  }

  setLFTestPhase(LF_TEST_STOP_BEFORE);
}

void printLFTestStatus(int16_t physicalCommand, int16_t rawLFCommand)
{
  long delta[ENCODER_COUNT];
  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    delta[i] = encoderCounts[i] - lfTestLastEncoderCounts[i];
  }

  Serial.print("LF_TEST | phase=");
  Serial.print(lfTestPhaseName(lfTestPhase));
  Serial.print(" cmdPhysical=");
  Serial.print(physicalCommand);
  Serial.print(" rawLF=");
  Serial.print(rawLFCommand);
  Serial.print(" | ENC RF=");
  Serial.print(encoderCounts[0]);
  Serial.print(" RR=");
  Serial.print(encoderCounts[1]);
  Serial.print(" LR=");
  Serial.print(encoderCounts[2]);
  Serial.print(" LF=");
  Serial.print(encoderCounts[3]);
  Serial.print(" | d100 RF=");
  Serial.print(delta[0]);
  Serial.print(" RR=");
  Serial.print(delta[1]);
  Serial.print(" LR=");
  Serial.print(delta[2]);
  Serial.print(" LF=");
  Serial.println(delta[3]);

  copyEncoderCounts(lfTestLastEncoderCounts);
}

void loopLFMotorEncoderTest()
{
  updateButtons();
  updateLed();
  pollEncoders();

  unsigned long nowMs = millis();

  if (killStopped)
  {
    stopAllMotors();
    if (nowMs - lastStatePrintMs >= 500)
    {
      lastStatePrintMs = nowMs;
      printLFTestStatus(0, 0);
    }
    delay(5);
    return;
  }

  unsigned long phaseElapsedMs = nowMs - lfTestPhaseStartMs;
  switch (lfTestPhase)
  {
    case LF_TEST_STOP_BEFORE:
      if (phaseElapsedMs >= LF_TEST_STOP_MS)
      {
        setLFTestPhase(LF_TEST_FORWARD);
      }
      break;
    case LF_TEST_FORWARD:
      if (phaseElapsedMs >= LF_TEST_RUN_MS)
      {
        setLFTestPhase(LF_TEST_STOP_MID);
      }
      break;
    case LF_TEST_STOP_MID:
      if (phaseElapsedMs >= LF_TEST_STOP_MS)
      {
        setLFTestPhase(LF_TEST_REVERSE);
      }
      break;
    case LF_TEST_REVERSE:
      if (phaseElapsedMs >= LF_TEST_RUN_MS)
      {
        setLFTestPhase(LF_TEST_STOP_AFTER);
      }
      break;
    case LF_TEST_STOP_AFTER:
      if (phaseElapsedMs >= LF_TEST_STOP_MS)
      {
        setLFTestPhase(LF_TEST_STOP_BEFORE);
      }
      break;
  }

  int16_t physicalCommand = lfTestPhysicalCommand();
  int16_t rawLFCommand = physicalCommand * FWD_LF;
  setMotorSpeeds(0, 0, 0, rawLFCommand);

  if (nowMs - lfTestLastPrintMs >= LF_TEST_PRINT_MS)
  {
    lfTestLastPrintMs = nowMs;
    printLFTestStatus(physicalCommand, rawLFCommand);
  }

  delay(2);
}

void setup()
{
  if (SERVO_TEST)
  {
    setupServoTest();
    return;
  }

  if (RFID_ONLY_TEST)
  {
    setupRFIDOnlyTest();
    return;
  }

  if (LF_MOTOR_ENCODER_TEST)
  {
    setupLFMotorEncoderTest();
    return;
  }

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
  setupServerMessenger();

  for (uint8_t i = 0; i < NUM_ULTRASONIC; i++)
  {
    pinMode(ultrasonicSensors[i].trigPin, OUTPUT);
    pinMode(ultrasonicSensors[i].echoPin, INPUT);
    digitalWrite(ultrasonicSensors[i].trigPin, LOW);
  }

  Wire.begin();
  Wire.setClock(400000);
  scanI2CBus(Wire, "Wire IMU bus");

  while (!initIMU())
  {
    Serial.println("IMU | Not ready, retrying...");
    delay(500);
  }

  Wire1.begin();
  Wire1.setClock(400000);
  scanI2CBus(Wire1, "Wire1 for Motorons");

  setupMotoron(mcLower, RF, RR, LR);
  setupMotoron(mcUpper, LF);

  stopAllMotors();
  setupSeedPlanter();

  if (USE_RFID)
  {
    rfidReady = initRFIDNoResetPin();
  }

  Serial.println("IMU | Keep car still for gyro calibration.");
  delay(1000);
  calibrateGyroZ();
  yawDeg = 0.0;
  turnStartYawDeg = 0.0;
  lastUpdateMicros = micros();
  captureTiltBaseline();

  Serial.println("Line follower ready. Lower=0x11, Upper=0x10, IMU yaw enabled.");
  Serial.print("MODE | ");
  Serial.println(OFFLINE_MODE ? "OFFLINE: server disabled, local RFID fallback." : "ONLINE: server enabled, RFID waits for server replies.");
  Serial.println("SPEED_GOV | Debug ON: req/out are motor commands; delta is encoder ticks per sample.");
  Serial.println("Correction enabled: 5 reflectance sensors; IMU right-angle turns enabled.");
  Serial.println("WALL | If no black line is detected, follow tunnel wall at 15cm until line returns.");
  Serial.println("GRID_OBS | Disabled in main. Use env:grid_obstacle_avoidance for standalone obstacle test.");
  Serial.println("MECH | Startup waits stopped: red slow blink=no WiFi/server, fast blink=connected. Press D45 once to run.");
  Serial.println("MECH | After startup, D45 toggles movement stop/run. D32 revives/resumes and sets LED green.");
  Serial.println("MECH | Emergency warning overrides LEDs with alternating red/green blink.");
  Serial.println(USE_RFID ? "RFID | Enabled on SDA1/SCL1, address 0x28." : "RFID | Disabled.");
  Serial.println("US | Front E37/T39, Left E52/T44, Right E46/T48.");
  delay(1000);

  setKillStopped(true, "Startup wait for D45");

  enterState(SENSE_LINE);
}

void loop()
{
  if (SERVO_TEST){
    loopServoTest();
    return;
  }

  if (RFID_ONLY_TEST)
  {
    loopRFIDOnlyTest();
    return;
  }

  if (LF_MOTOR_ENCODER_TEST)
  {
    loopLFMotorEncoderTest();
    return;
  }

  updateServerMessenger();
  updateButtons();
  updateLed();
  pollEncoders();
  updateYaw();
  updateRFID();
  updateSeedPlanter();
  updateUltrasonic();

  if (killStopped){
    stopAllMotors();
    printSensorStatus();
    delay(5);
    return;
  }

  if (serverStopActive())
  {
    stopAllMotors();
    if (millis() - serverLastStatusMs >= SERVER_STATUS_INTERVAL_MS){
      serverLastStatusMs = millis();
      Serial.print("SERVER | Movement held. heartbeat=");
      Serial.print(serverHeartbeatAllowsMovement ? "OK" : "STOP");
      Serial.print(" emergency=");
      Serial.print(serverEmergencyActive ? "ON" : "OFF");
      Serial.print(" disable=");
      Serial.println(serverDisableActive ? "ON" : "OFF");
    }
    printSensorStatus();
    delay(5);
    return;
  }

  if (returnPlanPending)
  {
    planEmergencyReturn();
  }

  if (obstacleAvoidanceAllowed() && handleFrontObstacle()){
    printSensorStatus();
    delay(5);
    return;
  }

  switch (state){
    case SENSE_LINE:
      senseLineStep();
      break;

    case FORWARD_PULSE:{
      if (USE_LINE_PID){
        linePidFollowStep();
        break;
      }

      int16_t forwardSpeed = postTurnFineLineActive()
                                 ? (fieldNavigationActive()
                                        ? FIELD_POST_TURN_FINE_FORWARD_SPEED
                                        : POST_TURN_FINE_FORWARD_SPEED)
                                 : (fieldNavigationActive() ? FIELD_FORWARD_SPEED : FORWARD_SPEED);
      uint16_t forwardPulseMs = postTurnFineLineActive() ? POST_TURN_FINE_FORWARD_PULSE_MS : FORWARD_PULSE_MS;

      driveForward(forwardSpeed);
      if (millis() - stateStartMs >= forwardPulseMs){
        stopAllMotors();
        enterState(SENSOR_SETTLE);
      }
      break;
    }

    case SENSOR_SETTLE:
      stopAllMotors();
      if (millis() - stateStartMs >=
          (returnBaseReacquireSettleActive ? RETURN_BASE_REACQUIRE_SETTLE_MS : SENSOR_SETTLE_MS)){
        if (returnBaseReacquireSettleActive)
        {
          returnBaseReacquireSettleActive = false;
          resetLinePid();
          Serial.println("RETURN | Base settle complete. PID line following active; corner detection temporarily locked.");
        }
        enterState(SENSE_LINE);
      }
      break;

    case PRE_CORRECT_STOP:
      stopAllMotors();
      
      if (millis() - stateStartMs >= PRE_CORRECT_STOP_MS){
        if (correctionDirection < 0){
          enterState(CORRECT_LEFT);
        }
        else if (correctionDirection > 0){
          enterState(CORRECT_RIGHT);
        }
        else{
          enterState(SENSE_LINE);
        }
      }
      break;

    case CORRECT_LEFT:

    case CORRECT_RIGHT:
      correctionStep();
      break;

    case AFTER_CORRECT_SETTLE:
      stopAllMotors();
      if (millis() - stateStartMs >= AFTER_CORRECT_SETTLE_MS)
      {
        enterState(SENSE_LINE);
      }
      break;

    case PRE_TURN_FORWARD_LEFT:
      driveForward(fieldNavigationActive() ? FIELD_FORWARD_SPEED : FORWARD_SPEED);
      if (millis() - stateStartMs >= activePreTurnForwardLeftMs())
      {
        stopAllMotors();
        enterState(TURN_LEFT_90);
      }
      break;

    case PRE_TURN_FORWARD_RIGHT:
      driveForward(fieldNavigationActive() ? FIELD_FORWARD_SPEED : FORWARD_SPEED);
      if (millis() - stateStartMs >= PRE_TURN_FORWARD_RIGHT_MS)
      {
        stopAllMotors();
        enterState(TURN_RIGHT_90);
      }
      break;

    case TURN_LEFT_90:
      rightAngleTurnStep(true);
      break;

    case TURN_RIGHT_90:
      rightAngleTurnStep(false);
      break;

    case TURN_AROUND_180:
      rightAngleTurnStep(activeTurnDirection < 0);
      break;

    case POST_TURN_FORWARD_VERIFY:
      postTurnForwardVerifyStep();
      break;

    case POST_TURN_BACK_UP:
      postTurnBackUpStep();
      break;

    case AFTER_TURN_SETTLE:
      stopAllMotors();
      if (millis() - stateStartMs >= AFTER_TURN_SETTLE_MS){
        enterFirstFindLineState();
      }
      break;
    case FIND_LINE_SMALL_LEFT:
      findLineStep(true, findLineSmallDuration(true), findLineSmallSweepDeg(true), nextSmallFindLineState(true));
      break;
    case FIND_LINE_SMALL_RIGHT:
      findLineStep(false, findLineSmallDuration(false), findLineSmallSweepDeg(false), nextSmallFindLineState(false));
      break;
    case FIND_LINE_BIG_LEFT:
      findLineStep(true, findLineBigDuration(true), findLineBigSweepDeg(true), nextBigFindLineState(true));
      break;
    case FIND_LINE_BIG_RIGHT:
      findLineStep(false, findLineBigDuration(false), findLineBigSweepDeg(false), nextBigFindLineState(false));
      break;
    case NO_LINE_RETURN_YAW:
      noLineReturnYawStep();
      break;
    case NO_LINE_IMU_FORWARD:
      noLineImuForwardStep();
      break;
    case WALL_FOLLOW:
      wallFollowStep();
      break;
    case GRID_OBSTACLE_AVOID:
      gridObstacleAvoidStep();
      break;
    case RFID_PAUSE:
      handleRFIDPauseState();
      break;
    case LINE_LOST_STOP:
      stopAllMotors();
      if (millis() - stateStartMs >= LINE_LOST_STOP_RECOVERY_MS)
      {
        startFixedRecovery("line_lost_stop");
      }
      break;
  }

  printSensorStatus();
  delay(5);
}
