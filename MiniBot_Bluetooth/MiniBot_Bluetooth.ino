
#include "auto_types.h"
#include "esp_task_wdt.h"
#include <ESP32Servo.h>
#include <FastLED.h>
#include <BluetoothSerial.h>
#include <esp_arduino_version.h>

BluetoothSerial RobotBT;

const uint8_t IO_PINS[13] = { 0, 32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21 };

#define User_BTN_A 35
#define User_BTN_B 39
#define User_BTN_C 38
#define User_BTN_D 37
#define User_SW    36

#define VIN_SENSE   34
#define VOLT_SLOPE  0.0063492
#define VOLT_OFFSET 1.079

#define LED_PIN     33
#define LED_COUNT   4
#define LED_PIN2    22
#define NUM_LEDS2   18
#define BRIGHTNESS  255

CRGB leds[LED_COUNT];
CRGB leds2[NUM_LEDS2];

#define WDT_TIMEOUT 3
Servo MotorOutput[13];

float battVoltage = 0;
bool  batteryWarn = false;
const float BATT_WARN_V = 7.2;

MotorTypeConfig motorTypeConfigs[] = {
  { MG90_CR,     50, 500,  2500 },
  { MG90_Degree, 50, 500,  2500 },
  { N20Plus,     50, 800,  2200 },
  { Victor_SPX,  50, 1000, 2000 },
  { Talon_SRX,   50, 1000, 2000 },
  { STD_SERVO,   50, 1000, 2000 },
  { SPARK_MAX,   50, 1000, 2000 },
};

int lastMotorPosition[13];
bool motorPositionWritten[13] = {};

void WriteMotor(uint8_t slot, int position) {
  if (slot >= 7 && slot <= 10) return;
  position = constrain(position, 0, 180);
  if (motorPositionWritten[slot] && lastMotorPosition[slot] == position) return;
  MotorOutput[slot].write(position);
  lastMotorPosition[slot] = position;
  motorPositionWritten[slot] = true;
}

void ConfigureMotorOutput(uint8_t slot, MotorType motorType, int startupPosition = 90) {
  if (slot >= 7 && slot <= 10) {
    pinMode(IO_PINS[slot], OUTPUT);
    digitalWrite(IO_PINS[slot], LOW);
    return;
  }
  float pwmFreq = 50;
  int minPulseUs = 1000;
  int maxPulseUs = 2000;

  for (auto &cfg : motorTypeConfigs) {
    if (cfg.type == motorType) {
      pwmFreq = cfg.pwmFreq;
      minPulseUs = cfg.minPulseUs;
      maxPulseUs = cfg.maxPulseUs;
      break;
    }
  }

  uint8_t pin = IO_PINS[slot];
  pinMode(pin, OUTPUT);
  MotorOutput[slot].setPeriodHertz(pwmFreq);
  MotorOutput[slot].attach(pin, minPulseUs, maxPulseUs);
  WriteMotor(slot, startupPosition);

  Serial.printf("Motor slot %d on pin %d, type %d\n", slot, pin, motorType);
}

bool manualActive = false;
float manualForward = 0;
float manualTurn = 0;
unsigned long lastManualMs = 0;
const unsigned long MANUAL_TIMEOUT_MS = 250;
float requestedForward = 0;
float requestedTurn = 0;
uint8_t lastCommand = 0;
uint32_t commandCount = 0;
const char *lastReason = "boot";
float currentLeftOut  = 90;
float currentRightOut = 90;
const float SLEW_RATE = 2.0f;
const float TURN_SLEW_RATE = 0.75f;
unsigned long leftReverseDwell = 0;
unsigned long rightReverseDwell = 0;
const unsigned long REVERSE_DWELL_MS = 100;

void SlewMove(float &current, float target, float step = SLEW_RATE) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
}

void SetDrive(float forward, float turn) {
  requestedForward = forward;
  requestedTurn = turn;
  if (fabsf(forward) < 0.01f && fabsf(turn) < 0.01f) {
    currentLeftOut = 90;
    currentRightOut = 90;
    for (int i = 1; i <= 6; i++) WriteMotor(i, 90);
    return;
  }
  float left  = constrain(-forward + turn, -100.0f, 100.0f);
  float right = constrain(-forward - turn, -100.0f, 100.0f);

  int MappedLeft  = constrain((int)(90.0f + 0.9f * left),  0, 180);
  int MappedRight = constrain((int)(90.0f - 0.9f * right), 0, 180);

  float outputStep = fabsf(forward) < 0.01f ? TURN_SLEW_RATE : SLEW_RATE;
  bool leftReversing  = (currentLeftOut  > 90 && MappedLeft  < 90) || (currentLeftOut  < 90 && MappedLeft  > 90);
  bool rightReversing = (currentRightOut > 90 && MappedRight < 90) || (currentRightOut < 90 && MappedRight > 90);

  if (leftReversing) {
    currentLeftOut = 90;
    leftReverseDwell = millis();
  } else if (millis() - leftReverseDwell > REVERSE_DWELL_MS) {
    SlewMove(currentLeftOut, MappedLeft, outputStep);
  }

  if (rightReversing) {
    currentRightOut = 90;
    rightReverseDwell = millis();
  } else if (millis() - rightReverseDwell > REVERSE_DWELL_MS) {
    SlewMove(currentRightOut, MappedRight, outputStep);
  }

  WriteMotor(1, currentRightOut);
  WriteMotor(2, currentRightOut);
  WriteMotor(3, currentRightOut);
  WriteMotor(4, currentLeftOut);
  WriteMotor(5, currentLeftOut);
  WriteMotor(6, currentLeftOut);
}

const int TRIM_7  = -7;
const int TRIM_8  = 0;
const int TRIM_9  = 0;
const int TRIM_10 = 0;
const int ARM_HOME     = 90;
const int SUPPORT_HOME = 3;

void HomeServos() {
  WriteMotor(7, constrain(ARM_HOME + TRIM_7, 0, 180));
  WriteMotor(8, constrain((180 - ARM_HOME) + TRIM_8, 0, 180));
  WriteMotor(9, constrain((180 - SUPPORT_HOME) + TRIM_9, 0, 180));
  WriteMotor(10, constrain(SUPPORT_HOME + TRIM_10, 0, 180));
}

void StopAll() {
  requestedForward = 0;
  requestedTurn = 0;
  currentLeftOut = 90;
  currentRightOut = 90;
  for (int i = 1; i <= 6; i++) WriteMotor(i, 90);
  WriteMotor(12, 90);
  HomeServos();
}

// !! VERIFY EVERY NUMBER BELOW AGAINST THE ONSHAPE ARENA CAD !!
const int16_t FIELD_X_MM = 1829;
const int16_t FIELD_Y_MM = 1219;

const Pose2D START_BLUE   = { 300, 610, 0 };
const Pose2D START_ORANGE = { 1529, 610, 180 };

const Pose2D CORNER_TARGETS[] = {
  { 305, 305, 0 },    // TGT_CORNER_BLUE_A   ~12 in, 12 in   VERIFY
  { 305, 914, 0 },    // TGT_CORNER_BLUE_B   ~12 in, 36 in   VERIFY
  { 1524, 305, 0 },   // TGT_CORNER_ORG_A    ~60 in, 12 in   VERIFY
  { 1524, 914, 0 },   // TGT_CORNER_ORG_B    ~60 in, 36 in   VERIFY
};

const CenterPlatform CENTER_PLATFORMS[2] = {
  { { 610, 610, 0 },
    { { { 410, 610, 0 },    0 },      // from the blue side, climb +X    VERIFY
      { { 810, 610, 180 },  180 },    // from the middle, climb -X       VERIFY
      { { 610, 360, 90 },   90 } } }, // from the side, climb +Y         VERIFY
  { { 1219, 610, 0 },
    { { { 1019, 610, 0 },   0 },      // VERIFY
      { { 1419, 610, 180 }, 180 },    // VERIFY
      { { 1219, 360, 90 },  90 } } }, // VERIFY
};

const Obstacle OBSTACLES[2] = {
  { 610,  610, 260 },
  { 1219, 610, 260 },
};

volatile bool linkPacketReady = false;
LinkPacket    rxPacket;
uint16_t      lastSeq = 0;
unsigned long lastPacketMs = 0;
unsigned long lastPoseMs   = 0;
unsigned long lastMacPrint = 0;

uint8_t LinkChecksum(const uint8_t *buf, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c ^= buf[i];
  return c;
}

uint8_t serBuf[sizeof(LinkPacket)];
int     serFill  = 0;
int     serState = 0;

void AcceptPacket(const LinkPacket &p) {
  if (p.magic != LINK_MAGIC || p.version != LINK_VERSION) return;
  if (LinkChecksum((const uint8_t *)&p, sizeof(p) - 1) != p.checksum) return;
  if (p.cmd > CMD_MANUAL || p.poseValid > 1 ||
      (p.cmd == CMD_MANUAL && (abs(p.gx_mm) > 24 || abs(p.gy_mm) > 22)) ||
      (p.poseValid && (p.heading_cdeg < -18000 || p.heading_cdeg > 18000)) ||
      ((p.cmd == CMD_GOTO_XY || p.cmd == CMD_PATH_ADD) &&
       (p.gx_mm < 0 || p.gx_mm > FIELD_X_MM || p.gy_mm < 0 || p.gy_mm > FIELD_Y_MM))) {
    Disable("invalid command");
    return;
  }
  rxPacket = p;
  linkPacketReady = true;
}

void PollSerialLink() {
  while (RobotBT.available()) {
    uint8_t b = RobotBT.read();
    if (serState == 0) {
      if (b == 0xAB) serState = 1;
    } else if (serState == 1) {
      serState = (b == 0xCD) ? 2 : 0;
      serFill = 0;
    } else {
      serBuf[serFill++] = b;
      if (serFill == (int)sizeof(LinkPacket)) {
        LinkPacket p;
        memcpy(&p, serBuf, sizeof(p));
        serState = 0;
        serFill = 0;
        AcceptPacket(p);
        return;
      }
    }
  }
}

AutoState state = ST_DISABLED;

Pose2D  pose         = { 0, 0, 0 };
bool    poseFresh    = false;
Pose2D  goal         = { 0, 0, 0 };
Pose2D  finalGoal    = { 0, 0, 0 };
bool    hasDetour    = false;
uint8_t activeTarget = TGT_NONE;
int     activeRamp   = -1;
bool    climbingTarget = false;

int16_t pathX[PATH_MAX];
int16_t pathY[PATH_MAX];
uint8_t pathCount     = 0;
uint8_t pathIndex     = 0;
bool    followingPath = false;
bool    legTurning    = false;
unsigned long pauseUntil = 0;
unsigned long headingStableSince = 0;
float bestLegDistance = 1e9f;
int travelDirection = 0;
unsigned long turnStartedMs = 0;

unsigned long holdStartMs  = 0;
unsigned long matchStartMs = 0;
bool          matchRunning = false;

const unsigned long POSE_TIMEOUT_MS = 300;
const unsigned long LINK_TIMEOUT_MS = 900;
const unsigned long MATCH_LIMIT_MS  = 90000;
const unsigned long CAPTURE_HOLD_MS = 3000;

const float ARRIVE_TOL_MM     = 80.0f;
const float WAYPOINT_TOL_MM   = 90.0f;
const float TURN_SETTLE_DEG   = 15.0f;
const float TURN_REDO_DEG     = 30.0f;
const float TURN_HOLD_DEG     = 25.0f;
const float DRIVE_HEADING_DEADZONE_DEG = 10.0f;
const float DRIVE_HEADING_MAX_DEADZONE_DEG = 20.0f;
const float TURN_SLOW_DEG     = 70.0f;
const float TURN_MIN_SPEED    = 18.0f;

const unsigned long SETTLE_MS   = 350;
const unsigned long LEG_PAUSE_MS = 300;
const float STAGING_TOL_MM    = 30.0f;
const float ALIGN_TOL_DEG     = 4.0f;
const float TURN_IN_PLACE_DEG = 25.0f;
const float DRIVE_SPEED       = 24.0f;
const float CLIMB_SPEED       = 24.0f;
const float TURN_SPEED        = 22.0f;
const float HEADING_KP        = 0.3f;
const float SLOWDOWN_MM       = 400.0f;
const float DRIVE_MIN_SPEED   = 18.0f;
const float OVERSHOOT_NEAR_MM = 140.0f;
const float OVERSHOOT_RISE_MM = 35.0f;

float NormalizeDeg(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float DistanceTo(int16_t x, int16_t y) {
  float dx = (float)x - (float)pose.x_mm;
  float dy = (float)y - (float)pose.y_mm;
  return sqrtf(dx * dx + dy * dy);
}

float TurnSpeedFor(float err) {
  float approach = constrain((fabsf(err) - TURN_SETTLE_DEG) /
    (TURN_SLOW_DEG - TURN_SETTLE_DEG), 0.0f, 1.0f);
  float blend = approach * approach * (3.0f - 2.0f * approach);
  float sp = TURN_MIN_SPEED + (TURN_SPEED - TURN_MIN_SPEED) * blend;
  return err > 0 ? sp : -sp;
}

float BearingTo(int16_t x, int16_t y) {
  float dx = (float)x - (float)pose.x_mm;
  float dy = (float)y - (float)pose.y_mm;
  return degrees(atan2f(dy, dx));
}

void EnterState(AutoState s) {
  state = s;
  if (s == ST_HOLD) holdStartMs = millis();
}

float SegmentPointDist(float ax, float ay, float bx, float by, float px, float py) {
  float vx = bx - ax, vy = by - ay;
  float wx = px - ax, wy = py - ay;
  float len2 = vx * vx + vy * vy;
  float t = (len2 > 0.0f) ? ((wx * vx + wy * vy) / len2) : 0.0f;
  t = constrain(t, 0.0f, 1.0f);
  float cx = ax + t * vx, cy = ay + t * vy;
  float dx = px - cx, dy = py - cy;
  return sqrtf(dx * dx + dy * dy);
}

bool FindDetour(int16_t gx, int16_t gy, int16_t &outX, int16_t &outY) {
  for (const Obstacle &o : OBSTACLES) {
    float d = SegmentPointDist(pose.x_mm, pose.y_mm, gx, gy, o.x_mm, o.y_mm);
    if (d >= o.radius_mm) continue;

    float dx = (float)gx - (float)pose.x_mm;
    float dy = (float)gy - (float)pose.y_mm;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1.0f) return false;
    dx /= len; dy /= len;

    float nx = -dy, ny = dx;
    float reach = o.radius_mm + 60.0f;

    float ax = o.x_mm + nx * reach, ay = o.y_mm + ny * reach;
    float bx = o.x_mm - nx * reach, by = o.y_mm - ny * reach;

    bool aOk = (ax > 150 && ax < FIELD_X_MM - 150 && ay > 150 && ay < FIELD_Y_MM - 150);
    bool bOk = (bx > 150 && bx < FIELD_X_MM - 150 && by > 150 && by < FIELD_Y_MM - 150);

    if (aOk && !bOk)      { outX = (int16_t)ax; outY = (int16_t)ay; return true; }
    if (bOk && !aOk)      { outX = (int16_t)bx; outY = (int16_t)by; return true; }
    if (!aOk && !bOk)     return false;

    if (DistanceTo((int16_t)ax, (int16_t)ay) <= DistanceTo((int16_t)bx, (int16_t)by)) {
      outX = (int16_t)ax; outY = (int16_t)ay;
    } else {
      outX = (int16_t)bx; outY = (int16_t)by;
    }
    return true;
  }
  return false;
}

void SetGoalWithDetour(int16_t gx, int16_t gy) {
  legTurning = true;
  headingStableSince = 0;
  bestLegDistance = 1e9f;
  travelDirection = 0;
  turnStartedMs = 0;
  finalGoal.x_mm = gx;
  finalGoal.y_mm = gy;

  int16_t dx, dy;
  if (FindDetour(gx, gy, dx, dy)) {
    goal.x_mm = dx;
    goal.y_mm = dy;
    hasDetour = true;
    Serial.printf("detour via %d,%d\n", dx, dy);
  } else {
    goal.x_mm = gx;
    goal.y_mm = gy;
    hasDetour = false;
  }
}

void AbortToIdle(const char *why) {
  manualActive = false;
  lastReason = why;
  Serial.printf("ABORT: %s\n", why);
  StopAll();
  activeTarget = TGT_NONE;
  activeRamp = -1;
  climbingTarget = false;
  hasDetour = false;
  followingPath = false;
  legTurning = false;
  pauseUntil = 0;
  EnterState(ST_IDLE);
}

void Disable(const char *why) {
  manualActive = false;
  lastReason = why;
  Serial.printf("DISABLE: %s\n", why);
  StopAll();
  activeTarget = TGT_NONE;
  activeRamp = -1;
  climbingTarget = false;
  hasDetour = false;
  followingPath = false;
  legTurning = false;
  pauseUntil = 0;
  matchRunning = false;
  state = ST_DISABLED;
}

int ChooseRamp(const CenterPlatform &p) {
  int best = 0;
  float bestCost = 1e9f;
  for (int i = 0; i < 3; i++) {
    float d = DistanceTo(p.ramps[i].staging.x_mm, p.ramps[i].staging.y_mm);
    float turn = fabsf(NormalizeDeg(p.ramps[i].climb_heading - pose.heading_deg));
    float cost = d + turn * 2.0f;
    if (cost < bestCost) {
      bestCost = cost;
      best = i;
    }
  }
  return best;
}

void StartGotoXY(int16_t x, int16_t y) {
  manualActive = false;
  activeTarget = TGT_NONE;
  climbingTarget = false;
  activeRamp = -1;
  followingPath = false;
  SetGoalWithDetour(x, y);
  Serial.printf("GOTO xy %d,%d\n", x, y);
  EnterState(ST_DRIVE);
}

void StartPathLeg() {
  activeTarget = TGT_NONE;
  climbingTarget = false;
  activeRamp = -1;
  hasDetour = false;
  legTurning = true;
  headingStableSince = 0;
  bestLegDistance = 1e9f;
  travelDirection = 0;
  turnStartedMs = 0;
  goal.x_mm = pathX[pathIndex];
  goal.y_mm = pathY[pathIndex];
  Serial.printf("path leg %u/%u -> %d,%d\n", pathIndex + 1, pathCount, goal.x_mm, goal.y_mm);
  EnterState(ST_DRIVE);
}

void StartPathRun() {
  manualActive = false;
  if (pathCount == 0) {
    Serial.println("RUN ignored - no path loaded");
    return;
  }
  pathIndex = 0;
  followingPath = true;
  Serial.printf("RUN path of %u points\n", pathCount);
  StartPathLeg();
}

void StartGotoTarget(uint8_t id) {
  manualActive = false;
  followingPath = false;
  if (id == TGT_CENTER_BLUE || id == TGT_CENTER_ORANGE) {
    const CenterPlatform &p = CENTER_PLATFORMS[id == TGT_CENTER_BLUE ? 0 : 1];
    activeRamp = ChooseRamp(p);
    activeTarget = id;
    climbingTarget = true;
    SetGoalWithDetour(p.ramps[activeRamp].staging.x_mm, p.ramps[activeRamp].staging.y_mm);
    Serial.printf("GOTO centre platform %d via ramp %d\n", id, activeRamp);
    EnterState(ST_DRIVE);
  } else if (id >= TGT_CORNER_BLUE_A && id <= TGT_CORNER_ORG_B) {
    activeTarget = id;
    climbingTarget = false;
    activeRamp = -1;
    const Pose2D &t = CORNER_TARGETS[id - TGT_CORNER_BLUE_A];
    SetGoalWithDetour(t.x_mm, t.y_mm);
    Serial.printf("GOTO corner platform %d\n", id);
    EnterState(ST_DRIVE);
  } else {
    Serial.printf("unknown target %d, ignoring\n", id);
  }
}

void HandleCommand(const LinkPacket &p) {
  lastCommand = p.cmd;
  commandCount++;
  switch (p.cmd) {
    case CMD_ENABLE:
      if (state == ST_DISABLED) {
        lastReason = "enabled";
        Serial.println("ENABLE - match clock running");
        matchStartMs = millis();
        matchRunning = true;
        EnterState(ST_IDLE);
      }
      break;
    case CMD_DISABLE:
      Disable("commanded");
      break;
    case CMD_CANCEL:
      if (state != ST_DISABLED) AbortToIdle("cancelled");
      break;
    case CMD_GOTO_XY:
      if (state != ST_DISABLED) StartGotoXY(p.gx_mm, p.gy_mm);
      break;
    case CMD_GOTO_TGT:
      if (state != ST_DISABLED) StartGotoTarget(p.targetId);
      break;
    case CMD_PATH_CLEAR:
      if (state != ST_DISABLED) AbortToIdle("path cleared");
      pathCount = 0;
      pathIndex = 0;
      followingPath = false;
      Serial.println("path cleared");
      break;
    case CMD_PATH_ADD:
      if (p.targetId == pathCount && pathCount < PATH_MAX) {
        pathX[pathCount] = p.gx_mm;
        pathY[pathCount] = p.gy_mm;
        pathCount++;
        Serial.printf("path point %u = %d,%d\n", p.targetId, p.gx_mm, p.gy_mm);
      } else {
        Serial.printf("path point %u out of order (have %u), ignored\n", p.targetId, pathCount);
      }
      break;
    case CMD_MANUAL:
      if (state != ST_DISABLED) {
        if (p.gx_mm == 0 && p.gy_mm == 0) {
          AbortToIdle("manual released");
        } else {
          if (!manualActive) AbortToIdle("manual control");
          manualForward = p.gx_mm;
          manualTurn = p.gy_mm;
          lastManualMs = millis();
          manualActive = true;
        }
      }
      break;
    case CMD_PATH_RUN:
      if (state != ST_DISABLED) StartPathRun();
      break;
    default:
      break;
  }
}

void StatusLeds() {
  CRGB c = state == ST_DISABLED ? CRGB(255, 0, 0) : CRGB(0, 80, 255);
  static bool shown = false;
  static CRGB previous;
  if (shown && c == previous) return;
  previous = c;
  shown = true;
  fill_solid(leds, LED_COUNT, c);
  fill_solid(leds2, NUM_LEDS2, c);
  FastLED.show();
}

void setup() {
  Serial.begin(115200);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t watchdog = {};
  watchdog.timeout_ms = WDT_TIMEOUT * 1000;
  watchdog.idle_core_mask = 0;
  watchdog.trigger_panic = true;
  if (esp_task_wdt_init(&watchdog) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&watchdog);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true);
#endif


  pinMode(User_BTN_A, INPUT);
  pinMode(User_BTN_B, INPUT);
  pinMode(User_BTN_C, INPUT);
  pinMode(User_BTN_D, INPUT);
  pinMode(User_SW, INPUT);
  pinMode(VIN_SENSE, INPUT);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.addLeds<WS2812B, LED_PIN2, GRB>(leds2, NUM_LEDS2);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  ConfigureMotorOutput(1, N20Plus);
  ConfigureMotorOutput(2, N20Plus);
  ConfigureMotorOutput(3, N20Plus);
  ConfigureMotorOutput(4, N20Plus);
  ConfigureMotorOutput(5, N20Plus);
  ConfigureMotorOutput(6, N20Plus);
  ConfigureMotorOutput(7, MG90_Degree, constrain(ARM_HOME + TRIM_7, 0, 180));
  ConfigureMotorOutput(8, MG90_Degree, constrain((180 - ARM_HOME) + TRIM_8, 0, 180));
  ConfigureMotorOutput(9, MG90_Degree, constrain((180 - SUPPORT_HOME) + TRIM_9, 0, 180));
  ConfigureMotorOutput(10, MG90_Degree, constrain(SUPPORT_HOME + TRIM_10, 0, 180));
  ConfigureMotorOutput(12, N20Plus);
  StopAll();

  RobotBT.enableSSP(false, false);
  if (!RobotBT.begin("MiniBot-Control")) {
    Serial.println("Bluetooth failed - reset required");
  }
  Serial.printf("reset reason: %d\n", (int)esp_reset_reason());
  Serial.println("Pair Windows with MiniBot-Control, then select its outgoing COM port on the control page.");

  esp_task_wdt_add(NULL);
  state = ST_DISABLED;
  Serial.println("MiniBot_Auto ready, waiting for CMD_ENABLE");
}

void loop() {
  esp_task_wdt_reset();
  if (!RobotBT.hasClient()) {
    if (state != ST_DISABLED) Disable("Bluetooth disconnected");
    serState = 0;
    serFill = 0;
  } else {
    PollSerialLink();
  }

  battVoltage = (analogRead(VIN_SENSE) * VOLT_SLOPE) + VOLT_OFFSET;
  batteryWarn = battVoltage < BATT_WARN_V;

  unsigned long now = millis();

  if (linkPacketReady) {
    linkPacketReady = false;
    LinkPacket p = rxPacket;

    if (p.seq != lastSeq) {
      lastSeq = p.seq;
      lastPacketMs = now;

      if (!p.poseValid) lastPoseMs = 0;
      if (p.poseValid) {
        pose.x_mm = p.x_mm;
        pose.y_mm = p.y_mm;
        pose.heading_deg = p.heading_cdeg / 100.0f;
        lastPoseMs = now;
      }
      if (p.cmd != CMD_NONE) HandleCommand(p);
    }
  }

  poseFresh = lastPoseMs != 0 && (now - lastPoseMs) < POSE_TIMEOUT_MS;

  if (state != ST_DISABLED) {
    if (now - lastPacketMs > LINK_TIMEOUT_MS) {
      Disable("link lost");
    } else if (matchRunning && (now - matchStartMs > MATCH_LIMIT_MS)) {
      Disable("match time expired");
    }
  }

  if (manualActive && state != ST_DISABLED) {
    if (now - lastManualMs >= MANUAL_TIMEOUT_MS) AbortToIdle("manual input timeout");
    else SetDrive(manualForward, manualTurn);
  } else switch (state) {
    case ST_DISABLED:
      StopAll();
      break;

    case ST_IDLE:
    case ST_DONE:
      SetDrive(0, 0);
      break;

    case ST_DRIVE: {
      if (!poseFresh) {
        SetDrive(0, 0);
        break;
      }
      if (millis() < pauseUntil) {
        SetDrive(0, 0);
        break;
      }

      bool lastLeg = !followingPath || (pathIndex + 1 >= pathCount);
      float dist = DistanceTo(goal.x_mm, goal.y_mm);
      float tol;
      if (hasDetour)            tol = 80.0f;
      else if (climbingTarget)  tol = STAGING_TOL_MM;
      else if (!lastLeg)        tol = WAYPOINT_TOL_MM;
      else                      tol = ARRIVE_TOL_MM;

      bool overshot = !climbingTarget && !hasDetour &&
        bestLegDistance <= OVERSHOOT_NEAR_MM && dist > bestLegDistance + OVERSHOOT_RISE_MM;
      if (dist < bestLegDistance) bestLegDistance = dist;
      if (dist <= tol || overshot) {
        SetDrive(0, 0);
        if (hasDetour) {
          SetGoalWithDetour(finalGoal.x_mm, finalGoal.y_mm);
          pauseUntil = millis() + LEG_PAUSE_MS;
          break;
        }
        if (!lastLeg) {
          pathIndex++;
          StartPathLeg();
          pauseUntil = millis() + LEG_PAUSE_MS;
          break;
        }
        followingPath = false;
        EnterState(climbingTarget ? ST_ALIGN : ST_HOLD);
        break;
      }

      float bearing = BearingTo(goal.x_mm, goal.y_mm);
      if (travelDirection == 0) {
        float forwardError = NormalizeDeg(bearing - pose.heading_deg);
        travelDirection = !climbingTarget && fabsf(forwardError) > 100.0f ? -1 : 1;
      }
      float desiredHeading = NormalizeDeg(bearing + (travelDirection < 0 ? 180.0f : 0.0f));
      float err = NormalizeDeg(desiredHeading - pose.heading_deg);

      if (legTurning) {
        if (turnStartedMs == 0) turnStartedMs = now;
        if (now - turnStartedMs > 7000) {
          AbortToIdle("turn did not align; check front direction or wheel traction");
          break;
        }
        if (fabsf(err) <= TURN_SETTLE_DEG ||
            (headingStableSince != 0 && fabsf(err) <= TURN_HOLD_DEG)) {
          SetDrive(0, 0);
          if (headingStableSince == 0) headingStableSince = now;
          if (now - headingStableSince >= SETTLE_MS) {
            legTurning = false;
            headingStableSince = 0;
            turnStartedMs = 0;
          }
        } else {
          headingStableSince = 0;
          SetDrive(0, TurnSpeedFor(err));
        }
        break;
      }

      if (fabsf(err) > TURN_REDO_DEG) {
        SetDrive(0, 0);
        legTurning = true;
        headingStableSince = 0;
        turnStartedMs = 0;
        break;
      }

      float approach = constrain((dist - tol) / (SLOWDOWN_MM - tol), 0.0f, 1.0f);
      float speed = DRIVE_MIN_SPEED + (DRIVE_SPEED - DRIVE_MIN_SPEED) * approach;
      float deadzone = constrain(degrees(atan2f(ARRIVE_TOL_MM, dist)),
        DRIVE_HEADING_DEADZONE_DEG, DRIVE_HEADING_MAX_DEADZONE_DEG);
      float trimLimit = min(4.0f, speed * 0.25f);
      float trimError = max(0.0f, fabsf(err) - deadzone);
      float trim = min(HEADING_KP * trimError, trimLimit) * (err < 0 ? -1.0f : 1.0f);
      SetDrive(travelDirection * speed, trim);
      break;
    }

    case ST_ALIGN: {
      if (!poseFresh || activeRamp < 0) {
        SetDrive(0, 0);
        break;
      }
      const CenterPlatform &p = CENTER_PLATFORMS[activeTarget == TGT_CENTER_BLUE ? 0 : 1];
      float want = p.ramps[activeRamp].climb_heading;
      float err  = NormalizeDeg(want - pose.heading_deg);

      if (millis() < pauseUntil) {
        SetDrive(0, 0);
        break;
      }

      if (fabsf(err) < ALIGN_TOL_DEG) {
        SetDrive(0, 0);
        if (!legTurning) {
          legTurning = true;
  headingStableSince = 0;
  bestLegDistance = 1e9f;
  travelDirection = 0;
  turnStartedMs = 0;
          pauseUntil = millis() + SETTLE_MS;
          break;
        }
        legTurning = false;
        EnterState(ST_CLIMB);
      } else {
        legTurning = false;
        SetDrive(0, TurnSpeedFor(err));
      }
      break;
    }

    case ST_CLIMB: {
      if (!poseFresh) {
        SetDrive(0, 0);
        break;
      }
      const CenterPlatform &p = CENTER_PLATFORMS[activeTarget == TGT_CENTER_BLUE ? 0 : 1];
      float dist = poseFresh ? DistanceTo(p.center.x_mm, p.center.y_mm) : 999.0f;

      if (poseFresh && dist < ARRIVE_TOL_MM) {
        SetDrive(0, 0);
        EnterState(ST_HOLD);
      } else {
        SetDrive(CLIMB_SPEED, 0);
      }
      break;
    }

    case ST_HOLD: {
      SetDrive(0, 0);
      if (now - holdStartMs >= CAPTURE_HOLD_MS) {
        Serial.println("hold complete");
        EnterState(ST_DONE);
      }
      break;
    }
  }

  static unsigned long lastTelemetry = 0;
  if (RobotBT.hasClient() && now - lastTelemetry >= 100) {
    lastTelemetry = now;
    uint8_t driveAttached = 0;
    for (int i = 1; i <= 6; i++) if (MotorOutput[i].attached()) driveAttached |= (1 << (i - 1));
    float headingError = state == ST_DRIVE ? NormalizeDeg(BearingTo(goal.x_mm, goal.y_mm) - pose.heading_deg) : 0.0f;
    RobotBT.printf("{\"state\":%d,\"pose_fresh\":%s,\"battery\":%.2f,\"seq\":%u,\"remaining_ms\":%lu,"
      "\"firmware\":\"bt-nav-8-manual\",\"manual\":%s,\"forward\":%.1f,\"turn\":%.1f,\"left_us\":%d,\"right_us\":%d,"
      "\"last_cmd\":%u,\"commands\":%lu,\"path_count\":%u,\"turning\":%s,\"heading_error\":%.1f,\"attached\":%u,\"reason\":\"%s\"}\n",
      (int)state, poseFresh ? "true" : "false", battVoltage, lastSeq,
      matchRunning ? (unsigned long)(MATCH_LIMIT_MS - min(now - matchStartMs, MATCH_LIMIT_MS)) : 0UL,
      manualActive ? "true" : "false", requestedForward, requestedTurn, MotorOutput[4].readMicroseconds(), MotorOutput[1].readMicroseconds(),
      lastCommand, (unsigned long)commandCount, pathCount, legTurning ? "true" : "false", headingError, driveAttached, lastReason);

  }
  StatusLeds();
  delay(20);
}
