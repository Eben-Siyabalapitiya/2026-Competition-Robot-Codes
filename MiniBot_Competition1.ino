#include <Bluepad32.h>
#include "esp_task_wdt.h"
#include <ESP32Servo.h>
#include <FastLED.h>

const float DRIVE_SCALE = 0.35;
const float TURN_SCALE  = 1.0;
const int   DEADBAND    = 50;

const float SLEW_RATE_PER_SEC = 1000.0;
const unsigned long REVERSE_DWELL_MS = 40;

const unsigned long CONTROL_INTERVAL_MS = 5;
const unsigned long LED_INTERVAL_MS     = 20;
const unsigned long BATT_SAMPLE_MS      = 100;
const unsigned long CONTROL_TIMEOUT_MS  = 300;

const int ARM_R1_ANGLE = 0;
const int ARM_R2_ANGLE = 70;
const int REST_BASE    = 115;

const int REST_TRIM_STEP = 2;
const int REST_TRIM_MIN  = -40;
const int REST_TRIM_MAX  = 40;

const int SUPPORT_ZERO = 0;
const int SUPPORT_MAX  = 150;

const int TRIM_7  = -7;
const int TRIM_8  = 0;
const int TRIM_9  = 0;
const int TRIM_10 = 0;

#define BRIGHTNESS 120
const int flashInterval = 50;
const int flashDurationCycles = 10;
int snakeSpeed = 35;
const unsigned long DPAD_EFFECT_DURATION = 8000;

const float BATT_WARN_V = 7.2;

int servoAngle = 90;
int supportAngle = 0;
int restTrim = 0;

#define LED_PIN2 22
#define NUM_LEDS2 18
CRGB leds2[NUM_LEDS2];

int snakePos = 0;
int snakeDirection = 1;
unsigned long lastSnakeMove = 0;
const int SNAKE_LEN = 3;

bool flashing = false;
unsigned long flashStart = 0;

enum FlashColor { FLASH_NONE, FLASH_WHITE, FLASH_RED };
FlashColor currentFlashColor = FLASH_NONE;

bool lastR2State = false;
bool lastR1State = false;
bool lastVictoryState = false;

bool victoryActive = false;
uint8_t victoryHue = 0;

bool lastDpadUp = false, lastDpadDown = false, lastDpadLeft = false;
enum DpadEffect { DPAD_NONE, DPAD_SIREN };
DpadEffect currentDpadEffect = DPAD_NONE;
unsigned long dpadEffectStart = 0;

float battVoltage = 0;
bool batteryWarn = false;

int ledBrightnessPercent = 100;
bool lastYState = false;
bool lastAState = false;

const uint8_t AUX_PINS[9] = { 0, 5, 18, 23, 19, 22, 21, 1, 3 };
const uint8_t IO_PINS[13] = { 0, 32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21 };

#define User_BTN_A 35
#define User_BTN_B 39
#define User_BTN_C 38
#define User_BTN_D 37
#define User_SW 36

#define VIN_SENSE 34
#define VOLT_SLOPE 0.0063492
#define VOLT_OFFSET 1.079

#define LED_PIN 33
#define LED_COUNT 4
#define COLOR_ORDER GRB
#define CHIPSET WS2812B

CRGB leds[LED_COUNT];
uint8_t rainbowHue = 0;

#define WDT_TIMEOUT 3
Servo MotorOutput[13];

float currentLeftOut = 90;
float currentRightOut = 90;

unsigned long leftReverseDwell = 0;
unsigned long rightReverseDwell = 0;

unsigned long lastControlUpdate = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastBattSample = 0;
unsigned long lastGoodPacket = 0;

void SlewMove(float &current, float target, float maxStep) {

  if (current < target) {
    current += maxStep;
    if (current > target)
      current = target;
  }

  else if (current > target) {
    current -= maxStep;
    if (current < target)
      current = target;
  }

}

void NeutralAllMotors() {
  for (int i = 1; i <= 12; i++) MotorOutput[i].write(90);
  currentLeftOut = 90;
  currentRightOut = 90;
}

void INIT_InternalFeatures() {
  Serial.begin(115200);

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  pinMode(User_BTN_A, INPUT);
  pinMode(User_BTN_B, INPUT);
  pinMode(User_BTN_C, INPUT);
  pinMode(User_BTN_D, INPUT);
  pinMode(User_SW, INPUT);
  pinMode(VIN_SENSE, INPUT);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);
  FastLED.addLeds<WS2812B, LED_PIN2, GRB>(leds2, NUM_LEDS2);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  delay(100);
}

ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("! GamePad connected !");
    myController = ctl;
    lastGoodPacket = millis();

    ctl->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
    ctl->setColorLED(0, 255, 0);

    BP32.enableNewBluetoothConnections(false);

    fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
    FastLED.show();

  } else {
    Serial.println("Another controller tried to connect but is rejected");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("! GamePad disconnected !");
    myController = nullptr;

    fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
    FastLED.show();
  }
}

static unsigned long lastBatteryUpdate = 0;
void GamePad_BatteryMonitor(unsigned long now) {
  if (now - lastBatteryUpdate > 1000) {
    int battery = myController->battery();
    if (battery == 0) {
      myController->setColorLED(255, 0, 0);

    } else if (battery <= 64) {
      myController->setColorLED(255, 0, 0);
      Serial.println("! GamePad Low Battery !");

      myController->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);

    } else if (battery <= 128) {
      myController->setColorLED(255, 255, 0);

    } else {
      myController->setColorLED(0, 255, 0);
    }
    lastBatteryUpdate = now;
  }
}

void INIT_BluetoothGamepad_PairMode() {
  if (!digitalRead(User_BTN_A) && !digitalRead(User_BTN_D)) {
    BP32.forgetBluetoothKeys();
    Serial.println("Gamepad Unpaird!");
    BP32.enableNewBluetoothConnections(true);
    BP32.setup(&onConnectedController, &onDisconnectedController);

    while (!(myController && myController->isConnected())) {
      esp_task_wdt_reset();

      fill_solid(leds, LED_COUNT, CRGB(0, 0, 255));
      FastLED.show();
      delay(100);
      fill_solid(leds, LED_COUNT, CRGB(255, 255, 255));
      FastLED.show();
      delay(100);

      BP32.update();
    }
    BP32.enableNewBluetoothConnections(false);
  } else BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.enableVirtualDevice(false);
}

void Powerup_Diagnostics_LED() {
  Serial.print("BOOT Condition: ");
  if (esp_reset_reason() == ESP_RST_TASK_WDT) {
    Serial.println("Watchdog Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(255, 255, 255));
  } else if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println("Brownout Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(255, 255, 0));
    delay(3000);
  } else if (esp_reset_reason() == ESP_RST_POWERON) {
    Serial.println("Power-on Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
  } else if (esp_reset_reason() == ESP_RST_SW) {
    Serial.println("Software Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(0, 0, 255));
  } else if (esp_reset_reason() == ESP_RST_PANIC) {
    Serial.println("Panic Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
  } else {
    Serial.println("UNKOWN Reset Detected");
    fill_solid(leds, LED_COUNT, CRGB(255, 0, 255));
  }

  FastLED.show();
  delay(500);
  fill_solid(leds, LED_COUNT, CRGB(0, 0, 0));
  FastLED.show();
  delay(100);
}

enum MotorType {
  MG90_CR,
  MG90_Degree,
  N20Plus,
  STD_SERVO,
  Victor_SPX,
  Talon_SRX,
  SPARK_MAX,
  CUSTOM
};

struct MotorTypeConfig {
  MotorType type;
  float pwmFreq;
  int minPulseUs;
  int maxPulseUs;
  float inputMin;
  float inputMax;
};

MotorTypeConfig motorTypeConfigs[] = {
  { MG90_CR, 50, 500, 2500, -1, 1 },
  { MG90_Degree, 50, 500, 2500, 1, 180 },
  { N20Plus, 50, 800, 2200, -1, 1 },
  { Victor_SPX, 50, 1000, 2000, -1, 1 },
  { Talon_SRX, 50, 1000, 2000, -1, 1 },
  { STD_SERVO, 50, 1000, 2000, -1, 1 },
  { SPARK_MAX, 50, 1000, 2000, -1, 1 },
};

void ConfigureMotorOutput(uint8_t slot, MotorType motorType, int startupPositionDeg = 90) {
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

  Serial.printf(
    "Motor slot %d configured on pin %d as type %d: freq=%.1f Hz, pulse=%d-%d us, start=%d deg\n",
    slot, pin, motorType, pwmFreq, minPulseUs, maxPulseUs, startupPositionDeg);

  }

void DpadSiren() {
  unsigned long elapsed = millis() - dpadEffectStart;
  bool redSide = (elapsed / 150) % 2 == 0;
  for (int i = 0; i < NUM_LEDS2; i++) {
    bool firstHalf = i < NUM_LEDS2 / 2;
    leds2[i] = (firstHalf == redSide) ? CRGB::Red : CRGB::Blue;
  }
  for (int i = 0; i < LED_COUNT; i++) {
    bool firstHalf = i < LED_COUNT / 2;
    leds[i] = (firstHalf == redSide) ? CRGB::Red : CRGB::Blue;
  }
}

void UpdateExternalStrip(bool r2Pressed, bool r1Pressed, bool dpadLeft, bool dpadRight) {


  if ((r2Pressed || r1Pressed) && victoryActive) {
    victoryActive = false;
  }

  if ((r2Pressed || r1Pressed) && currentDpadEffect != DPAD_NONE) {
    currentDpadEffect = DPAD_NONE;
  }

  if (dpadRight && !lastVictoryState && !victoryActive) {
    victoryActive = true;
    currentDpadEffect = DPAD_NONE;
  }
  lastVictoryState = dpadRight;

  if (dpadLeft && !lastDpadLeft) { currentDpadEffect = DPAD_SIREN; dpadEffectStart = millis(); }
  lastDpadLeft = dpadLeft;

  if (currentDpadEffect != DPAD_NONE) {
    if (millis() - dpadEffectStart > DPAD_EFFECT_DURATION) {
      currentDpadEffect = DPAD_NONE;
    } else {
      switch (currentDpadEffect) {
        case DPAD_SIREN: DpadSiren(); break;
        default: break;
      }
      return;
    }
  }

  if (r2Pressed && !lastR2State) {
    flashing = true;
    flashStart = millis();
    currentFlashColor = FLASH_WHITE;
  }
  if (r1Pressed && !lastR1State) {
    flashing = true;
    flashStart = millis();
    currentFlashColor = FLASH_RED;
  }
  lastR2State = r2Pressed;
  lastR1State = r1Pressed;

  if (victoryActive) {
    fill_rainbow(leds2, NUM_LEDS2, victoryHue, 12);
    fill_rainbow(leds, LED_COUNT, victoryHue, 40);
    victoryHue += 4;
    if (random8() < 40) {
      leds2[random16(NUM_LEDS2)] += CRGB::White;
      leds[random16(LED_COUNT)] += CRGB::White;
    }
    return;
  }

  if (flashing) {
    unsigned long elapsed = millis() - flashStart;
    if (elapsed < flashInterval * flashDurationCycles) {
      bool on = (elapsed / flashInterval) % 2 == 0;
      CRGB color = (currentFlashColor == FLASH_RED) ? CRGB::Red : CRGB::White;
      fill_solid(leds2, NUM_LEDS2, on ? color : CRGB::Black);
      fill_solid(leds, LED_COUNT, on ? color : CRGB::Black);
    } else {
      flashing = false;
      currentFlashColor = FLASH_NONE;
    }
  } else {
    if (millis() - lastSnakeMove > snakeSpeed) {
      fill_solid(leds2, NUM_LEDS2, CRGB::Black);
      fill_solid(leds, LED_COUNT, CRGB::Black);
      CRGB snakeColor = batteryWarn ? CRGB::Red : CRGB::White;
      for (int i = 0; i < SNAKE_LEN; i++) {
        int idx = snakePos + i;
        if (idx >= 0 && idx < NUM_LEDS2) leds2[idx] = snakeColor;
      }
      leds[snakePos % LED_COUNT] = snakeColor;
      snakePos += snakeDirection;
      if (snakePos + SNAKE_LEN - 1 >= NUM_LEDS2 - 1 || snakePos <= 0) {
        snakeDirection *= -1;
      }
      lastSnakeMove = millis();
    }
  }
}


void INIT_LoRcore() {
  INIT_InternalFeatures();
  Powerup_Diagnostics_LED();
  INIT_BluetoothGamepad_PairMode();
}

void setup() {
  INIT_LoRcore();

  Serial.println("Motors Startup");

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

  Serial.println("LoRcore V3 System Ready! ");
}

void loop() {
  esp_task_wdt_reset();
  BP32.update();

  unsigned long now = millis();

  if (now - lastBattSample >= BATT_SAMPLE_MS) {
    battVoltage = (analogRead(VIN_SENSE) * VOLT_SLOPE) + VOLT_OFFSET;
    batteryWarn = battVoltage < BATT_WARN_V;
    lastBattSample = now;
  }

  bool connected = myController && myController->isConnected();
  if (connected && myController->hasData()) {
    lastGoodPacket = now;
  }
  bool linkAlive = connected && (now - lastGoodPacket) < CONTROL_TIMEOUT_MS;

  if (linkAlive) {

    uint8_t dpad = myController->dpad();
    bool dpadUp    = dpad & DPAD_UP;
    bool dpadDown  = dpad & DPAD_DOWN;
    bool dpadLeft  = dpad & DPAD_LEFT;
    bool dpadRight = dpad & DPAD_RIGHT;

    if (dpadUp && !lastDpadUp) {
      restTrim = constrain(restTrim - REST_TRIM_STEP, REST_TRIM_MIN, REST_TRIM_MAX);
      Serial.printf("rest trim: %d\n", restTrim);
    }
    if (dpadDown && !lastDpadDown) {
      restTrim = constrain(restTrim + REST_TRIM_STEP, REST_TRIM_MIN, REST_TRIM_MAX);
      Serial.printf("rest trim: %d\n", restTrim);
    }
    lastDpadUp = dpadUp;
    lastDpadDown = dpadDown;

    bool yPressed = myController->y();
    bool aPressed = myController->a();

    if (yPressed && !lastYState) {
      ledBrightnessPercent = min(100, ledBrightnessPercent + 20);
    }
    if (aPressed && !lastAState) {
      ledBrightnessPercent = max(0, ledBrightnessPercent - 20);
    }
    lastYState = yPressed;
    lastAState = aPressed;

    if (now - lastControlUpdate >= CONTROL_INTERVAL_MS) {
      float dt = (now - lastControlUpdate) / 1000.0f;
      lastControlUpdate = now;
      float maxStep = SLEW_RATE_PER_SEC * dt;

      if (myController->r1()) {
        servoAngle = ARM_R1_ANGLE;
      } else if (myController->r2() || myController->l2()) {
        servoAngle = ARM_R2_ANGLE;
      } else {
        servoAngle = constrain(REST_BASE + restTrim, 0, 180);
      }
      MotorOutput[7].write(constrain(servoAngle + TRIM_7, 0, 180));
      MotorOutput[8].write(constrain((180 - servoAngle) + TRIM_8, 0, 180));

      if (myController->r2() || myController->l1()) {
        supportAngle = SUPPORT_MAX;
      } else {
        supportAngle = SUPPORT_ZERO;
      }
      MotorOutput[9].write(constrain((180 - supportAngle) + TRIM_9, 0, 180));
      MotorOutput[10].write(constrain(supportAngle + TRIM_10, 0, 180));

      int rawDrive = myController->axisRX();
      int rawTurn  = -myController->axisY();

      if (abs(rawDrive) < DEADBAND) rawDrive = 0;
      if (abs(rawTurn) < DEADBAND) rawTurn = 0;

      int moveValue = rawDrive * DRIVE_SCALE;
      int turnValue = rawTurn * TURN_SCALE;

      int left = moveValue + turnValue;
      int right = moveValue - turnValue;

      int MappedLeft = constrain(map(-left, -512, 512, 0, 180), 0, 180);
      int MappedRight = constrain(map(-right, -512, 512, 0, 180), 0, 180);

      bool leftReversing = (currentLeftOut > 90 && MappedLeft < 90) || (currentLeftOut < 90 && MappedLeft > 90);
      bool rightReversing = (currentRightOut > 90 && MappedRight < 90) || (currentRightOut < 90 && MappedRight > 90);

      if (leftReversing) {
        currentLeftOut = 90;
        leftReverseDwell = now;
      } else if (now - leftReverseDwell > REVERSE_DWELL_MS) {
        SlewMove(currentLeftOut, MappedLeft, maxStep);
      }

      if (rightReversing) {
        currentRightOut = 90;
        rightReverseDwell = now;
      } else if (now - rightReverseDwell > REVERSE_DWELL_MS) {
        SlewMove(currentRightOut, MappedRight, maxStep);
      }

      MotorOutput[1].write(currentRightOut);
      MotorOutput[2].write(currentRightOut);
      MotorOutput[3].write(currentRightOut);

      MotorOutput[4].write(currentLeftOut);
      MotorOutput[5].write(currentLeftOut);
      MotorOutput[6].write(currentLeftOut);

      GamePad_BatteryMonitor(now);
    }

    if (now - lastLedUpdate >= LED_INTERVAL_MS) {
      lastLedUpdate = now;
      FastLED.setBrightness(map(ledBrightnessPercent, 0, 100, 0, 255));
      UpdateExternalStrip(myController->r2(), myController->r1(), dpadLeft, dpadRight);
      FastLED.show();
    }

  } else {
    NeutralAllMotors();

    if (now - lastLedUpdate >= LED_INTERVAL_MS) {
      lastLedUpdate = now;
      FastLED.setBrightness(BRIGHTNESS);
      if (connected) {
        fill_solid(leds, LED_COUNT, CRGB(255, 60, 0));
        fill_solid(leds2, NUM_LEDS2, CRGB(255, 60, 0));
      } else {
        fill_solid(leds, LED_COUNT, CRGB(0, 80, 255));
        fill_solid(leds2, NUM_LEDS2, CRGB(0, 0, 0));
      }
      FastLED.show();
    }
  }
}
