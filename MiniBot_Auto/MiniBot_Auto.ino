
#include "auto_types.h"
#include "esp_task_wdt.h"
#include <ESP32Servo.h>
#include <FastLED.h>
#include <WiFi.h>
#include <esp_now.h>

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

void ConfigureMotorOutput(uint8_t slot, MotorType motorType) {
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
  MotorOutput[slot].writeMicroseconds(1500);

  Serial.printf("Motor slot %d on pin %d, type %d\n", slot, pin, motorType);
}

float currentLeftOut  = 90;
float currentRightOut = 90;
const float SLEW_RATE = 25;
unsigned long leftReverseDwell = 0;
unsigned long rightReverseDwell = 0;
const unsigned long REVERSE_DWELL_MS = 100;

void SlewMove(float &current, float target) {
  if (current < target) {
    current += SLEW_RATE;
    if (current > target) current = target;
  } else if (current > target) {
    current -= SLEW_RATE;
    if (current < target) current = target;
  }
}

void SetDrive(float forward, float turn) {
  float left  = constrain(forward + turn, -100.0f, 100.0f);
  float right = constrain(forward - turn, -100.0f, 100.0f);

  int MappedLeft  = constrain((int)(90.0f - 0.9f * left),  0, 180);
  int MappedRight = constrain((int)(90.0f - 0.9f * right), 0, 180);

  bool leftReversing  = (currentLeftOut  > 90 && MappedLeft  < 90) || (currentLeftOut  < 90 && MappedLeft  > 90);
  bool rightReversing = (currentRightOut > 90 && MappedRight < 90) || (currentRightOut < 90 && MappedRight > 90);

  if (leftReversing) {
    currentLeftOut = 90;
    leftReverseDwell = millis();
  } else if (millis() - leftReverseDwell > REVERSE_DWELL_MS) {
    SlewMove(currentLeftOut, MappedLeft);
  }

  if (rightReversing) {
    currentRightOut = 90;
    rightReverseDwell = millis();
  } else if (millis() - rightReverseDwell > REVERSE_DWELL_MS) {
    SlewMove(currentRightOut, MappedRight);
  }

  MotorOutput[1].write(currentRightOut);
  MotorOutput[2].write(currentRightOut);
  MotorOutput[3].write(currentRightOut);
  MotorOutput[4].write(currentLeftOut);
  MotorOutput[5].write(currentLeftOut);
  MotorOutput[6].write(currentLeftOut);
}

void StopAll() {
  currentLeftOut = 90;
  currentRightOut = 90;
  for (int i = 1; i <= 12; i++) MotorOutput[i].write(90);
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

void OnEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != (int)sizeof(LinkPacket)) return;

  LinkPacket p;
  memcpy(&p, data, sizeof(p));

  if (p.magic != LINK_MAGIC || p.version != LINK_VERSION) return;
  if (LinkChecksum((const uint8_t *)&p, sizeof(p) - 1) != p.checksum) return;

  rxPacket = p;
  linkPacketReady = true;
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

unsigned long holdStartMs  = 0;
unsigned long matchStartMs = 0;
bool          matchRunning = false;

const unsigned long POSE_TIMEOUT_MS = 300;
const unsigned long LINK_TIMEOUT_MS = 500;
const unsigned long MATCH_LIMIT_MS  = 90000;
const unsigned long CAPTURE_HOLD_MS = 3000;

const float ARRIVE_TOL_MM     = 40.0f;
const float STAGING_TOL_MM    = 30.0f;
const float ALIGN_TOL_DEG     = 4.0f;
const float TURN_IN_PLACE_DEG = 25.0f;
const float DRIVE_SPEED       = 55.0f;
const float CLIMB_SPEED       = 80.0f;
const float TURN_SPEED        = 45.0f;
const float HEADING_KP        = 1.6f;

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
  Serial.printf("ABORT: %s\n", why);
  StopAll();
  activeTarget = TGT_NONE;
  activeRamp = -1;
  climbingTarget = false;
  hasDetour = false;
  EnterState(ST_IDLE);
}

void Disable(const char *why) {
  Serial.printf("DISABLE: %s\n", why);
  StopAll();
  activeTarget = TGT_NONE;
  activeRamp = -1;
  climbingTarget = false;
  hasDetour = false;
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
  activeTarget = TGT_NONE;
  climbingTarget = false;
  activeRamp = -1;
  SetGoalWithDetour(x, y);
  Serial.printf("GOTO xy %d,%d\n", x, y);
  EnterState(ST_DRIVE);
}

void StartGotoTarget(uint8_t id) {
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
  switch (p.cmd) {
    case CMD_ENABLE:
      if (state == ST_DISABLED) {
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
    default:
      break;
  }
}

void StatusLeds() {
  CRGB c;
  switch (state) {
    case ST_DISABLED: c = CRGB(255, 0, 0);     break;
    case ST_IDLE:     c = CRGB(0, 80, 255);    break;
    case ST_DRIVE:    c = CRGB(0, 255, 0);     break;
    case ST_ALIGN:    c = CRGB(255, 180, 0);   break;
    case ST_CLIMB:    c = CRGB(255, 0, 255);   break;
    case ST_HOLD:     c = CRGB(255, 255, 255); break;
    case ST_DONE:     c = CRGB(0, 255, 120);   break;
  }
  if (batteryWarn) c = CRGB(255, 0, 0);
  if (!poseFresh && state != ST_DISABLED) {
    bool on = (millis() / 100) % 2 == 0;
    if (!on) c = CRGB(0, 0, 0);
  }
  fill_solid(leds, LED_COUNT, c);
  fill_solid(leds2, NUM_LEDS2, c);
  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

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
  ConfigureMotorOutput(7, MG90_Degree);
  ConfigureMotorOutput(8, MG90_Degree);
  ConfigureMotorOutput(9, MG90_Degree);
  ConfigureMotorOutput(10, MG90_Degree);
  ConfigureMotorOutput(12, N20Plus);
  StopAll();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("robot MAC (put this in the dongle sketch): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED - staying disabled");
  } else {
    esp_now_register_recv_cb(OnEspNowRecv);
    Serial.println("ESP-NOW ready");
  }

  state = ST_DISABLED;
  Serial.println("MiniBot_Auto ready, waiting for CMD_ENABLE");
}

void loop() {
  esp_task_wdt_reset();

  battVoltage = (analogRead(VIN_SENSE) * VOLT_SLOPE) + VOLT_OFFSET;
  batteryWarn = battVoltage < BATT_WARN_V;

  unsigned long now = millis();

  if (linkPacketReady) {
    linkPacketReady = false;
    LinkPacket p = rxPacket;

    if (p.seq != lastSeq) {
      lastSeq = p.seq;
      lastPacketMs = now;

      if (p.poseValid) {
        pose.x_mm = p.x_mm;
        pose.y_mm = p.y_mm;
        pose.heading_deg = p.heading_cdeg / 100.0f;
        lastPoseMs = now;
      }
      if (p.cmd != CMD_NONE) HandleCommand(p);
    }
  }

  poseFresh = (now - lastPoseMs) < POSE_TIMEOUT_MS;

  if (state != ST_DISABLED) {
    if (now - lastPacketMs > LINK_TIMEOUT_MS) {
      Disable("link lost");
    } else if (matchRunning && (now - matchStartMs > MATCH_LIMIT_MS)) {
      Disable("match time expired");
    }
  }

  switch (state) {
    case ST_DISABLED:
      StopAll();
      if (lastPacketMs == 0 && now - lastMacPrint > 3000) {
        lastMacPrint = now;
        Serial.print("robot MAC for ROBOT_MAC_STR in the dongle sketch: ");
        Serial.println(WiFi.macAddress());
      }
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
      float dist = DistanceTo(goal.x_mm, goal.y_mm);
      float tol  = hasDetour ? 80.0f : (climbingTarget ? STAGING_TOL_MM : ARRIVE_TOL_MM);

      if (dist < tol) {
        if (hasDetour) {
          SetGoalWithDetour(finalGoal.x_mm, finalGoal.y_mm);
          break;
        }
        SetDrive(0, 0);
        EnterState(climbingTarget ? ST_ALIGN : ST_HOLD);
        break;
      }

      float err = NormalizeDeg(BearingTo(goal.x_mm, goal.y_mm) - pose.heading_deg);
      if (fabsf(err) > TURN_IN_PLACE_DEG) {
        SetDrive(0, (err > 0 ? TURN_SPEED : -TURN_SPEED));
      } else {
        SetDrive(DRIVE_SPEED, constrain(HEADING_KP * err, -40.0f, 40.0f));
      }
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

      if (fabsf(err) < ALIGN_TOL_DEG) {
        SetDrive(0, 0);
        EnterState(ST_CLIMB);
      } else {
        SetDrive(0, (err > 0 ? TURN_SPEED : -TURN_SPEED));
      }
      break;
    }

    case ST_CLIMB: {
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

  StatusLeds();
  delay(20);
}
