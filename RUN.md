# Running it

Everything lives in `C:\Users\budhu\MiniBot\`.

## Every time you want the control page

Open a terminal in `C:\Users\budhu\MiniBot\2026-Competition-Robot-Codes` and run:

```
C:\Users\budhu\AppData\Local\Programs\Python\Python312\python.exe vision\web_ui.py
```

Once the dongle exists, add its port:

```
C:\Users\budhu\AppData\Local\Programs\Python\Python312\python.exe vision\web_ui.py --port COM5
```

Takes 10-20 seconds while the camera starts, then prints `open http://localhost:8000`.
Open that in a browser. Ctrl+C in the terminal to stop, or the camera stays claimed.

Or just double-click `MiniBot\shortcuts\MiniBot Control.bat`, which runs the same thing.

## What you do on the page

Left half is the camera, right half is a live map of the field. The map is a view
only - it cannot command the robot. All input happens on the camera.

1. **Calibrate.** Stand the robot on each of 4 known field points, press Capture
   each time. The status pill goes green at 4/4. Nothing works before this.
2. **Enable** (space). The 90 s match clock starts.
3. **Draw a path** (P). Click the dot on the AprilTag, move to stretch the line,
   click to place a point. Keep clicking to chain turns. Z undoes a point,
   Esc leaves draw mode.
4. **Run** (R). The robot drives the path itself.
5. Or skip the path: click anywhere on the camera to drive there, or press 1-6
   for a platform.

C cancels the current goal, D disables.

## Flashing the two boards

Use the launchers in `MiniBot\shortcuts\`. Each lists the COM ports and picks the
right board type for you.

| Board | Launcher | Board type |
|---|---|---|
| LoR Core V3 - autonomy | `Flash Robot - AUTO.bat` | `esp32-bluepad32:esp32:esp32` |
| LoR Core V3 - teleop | `Flash Robot - TELEOP.bat` | `esp32-bluepad32:esp32:esp32` |
| Spare ESP32 - dongle | `Flash Dongle.bat` | plain `esp32:esp32:esp32` |

### The dongle is not ready to flash yet

`ROBOT_MAC_STR` in `EspNowDongle/EspNowDongle.ino` is still the placeholder
`AA:BB:CC:DD:EE:FF`. The dongle checks for this and refuses to forward anything,
fast-blinking its LED, so flashing it now gives you a board that does nothing.

Order:

1. Flash the robot with `Flash Robot - AUTO.bat`.
2. Serial Monitor at 115200. It prints
   `robot MAC (put this in the dongle sketch): XX:XX:XX:XX:XX:XX`
   and reprints every 3 s until it hears a packet.
3. Paste that into `ROBOT_MAC_STR` at the top of `EspNowDongle.ino`.
4. Flash the dongle with `Flash Dongle.bat` - it refuses to run while the
   placeholder is still there, so it will tell you if you skipped step 3.
5. Leave the dongle plugged into the laptop. That USB is the data link, not just
   power. Solid LED means packets are flowing; it also prints
   `# sent N failed M` every 2 s.

The robot needs its battery for anything that moves. USB alone runs the logic but
browns out under six motors.
