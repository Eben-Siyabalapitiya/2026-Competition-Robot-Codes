# MiniBot 2026 — Laptop Setup

Everything needed to build and flash `MiniBot_Competition1.ino` on the LoR Core V3.
Versions below match Lord of Robots' official `LoR_Core_V3_MiniBot_Example` and were
compile-verified against this sketch.

## 1. Arduino IDE

Arduino IDE 2.x: https://www.arduino.cc/en/software

## 2. Board packages — install BOTH

File > Preferences > Additional boards manager URLs, add both lines:

    https://dl.espressif.com/dl/package_esp32_index.json
    https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json

Then Tools > Board > Boards Manager and install both:

- **esp32** by Espressif Systems
- **esp32_bluepad32** by Ricardo Quesada (may show as "ESP32 + Bluepad32")

Both are required — the Bluepad32 package pulls its compiler toolchain from the
Espressif package, so installing it alone fails.

Select **Tools > Board > esp32_bluepad32 > ESP32 Dev Module**. Make sure the
"ESP32 Dev Module" you pick is the one under the Bluepad32 group — the standard
`esp32 > ESP32 Dev Module` will not provide `Bluepad32.h`.

`Bluepad32.h` and `esp_task_wdt.h` both come from the core. Do **not** install any
"Bluepad32" entry from Library Manager.

## 3. Libraries

Tools > Manage Libraries:

| Library    | Version                | Notes |
|------------|------------------------|-------|
| ESP32Servo | **3.0.7 — pin this**   | Pick 3.0.7 from the version dropdown. Do not take updates. |
| FastLED    | latest (3.10.8 tested) | |

**ESP32Servo must stay at 3.0.7.** Lord of Robots pins it to avoid PWM/LEDC timing
regressions in newer releases. Newer versions still compile, so the IDE will not warn
you — the symptom is wrong motor timing at runtime, which is exactly what you do not
want to discover at a competition. If the IDE offers to update it, decline.

## 4. Upload

1. Connect the LoR Core V3 with a USB **data** cable (charge-only cables won't enumerate).
2. Tools > Port, select the board's port (`COM*` on Windows).
3. Verify, then Upload. First compile can take 3–5 minutes.
4. Serial Monitor at **115200 baud**; press RST to see startup output.

If it hangs on `Connecting...`: hold **BOOT**, tap **RST**, release **BOOT**, retry Upload.
No port on Windows? Install the CH340 driver: https://www.wch-ic.com/download/file?id=65

## 5. Pair the gamepad

1. Power on the LoR Core V3.
2. Hold **User Button A + User Button D**.
3. Still holding, press and release **RST**.
4. LEDs flash blue/white — release A and D.
5. Put the controller into its pairing mode.
6. Green flash = connected. It reconnects automatically on later power cycles.

This clears previously saved Bluetooth keys. Supported controllers:
https://bluepad32.readthedocs.io/en/latest/supported_gamepads/

## LED status

| LED | Meaning |
|-----|---------|
| Icy blue | Waiting for gamepad |
| Flashing blue/white | Pairing mode |
| Green flash | Gamepad connected |
| Rainbow | Receiving control data |
| Red | Disconnect or battery warning — check Serial Monitor |

## API reference

- `ConfigureMotorOutput(slot, type)` — set per-slot output type in `setup()`
- `MotorSpeed_Set(slot, speed)` — −100..100, 0 stops (N20Plus, continuous servos)
- `MotorPosition_Set(slot, angle)` — 0..180 (positional servos)

Speed and position calls are ignored on slots configured for the other command type.

Your sketch's slot map: 1–3 right drive, 4–6 left drive (both `N20Plus`),
7–8 arm and 9–10 support (`MG90_Degree`), 12 spare (`N20Plus`).

## Verified

Compiled against FQBN `esp32-bluepad32:esp32:esp32`, Bluepad32 core 4.1.0,
ESP32Servo 3.0.7, FastLED 3.10.8:

    Sketch uses 778933 bytes (59%) of program storage space. Maximum is 1310720 bytes.
    Global variables use 89412 bytes (27%) of dynamic memory, leaving 238268 bytes
    for local variables. Maximum is 327680 bytes.

Zero errors, zero warnings.

## Source

Setup details cross-checked against https://github.com/LordofRobots/LoR_Core_V3_MiniBot_Example
