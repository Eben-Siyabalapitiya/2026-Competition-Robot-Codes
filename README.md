# 2026 Competition Robot Codes

Robot code for all 2026 competitions. Each sketch is the full firmware for that
event, covering LoR Core V3, MiniBot, and any auxiliary systems.

Carried over from the 2025 archive so old setups are easy to reference and pull
snippets from.

## Sketches

### `MiniBot_Competition1/`

The competition firmware. This is what goes on the robot for a match.

Starts from the 2025 comp 4 code with the drive sides swapped (slots 1-3 right,
4-6 left) and both drive axes inverted to match the new motor mounting. Gamepad
control over Bluetooth, no radio other than the controller link.

All tuning values sit in one block at the top of the file.

Controls:

| Input | Action |
| --- | --- |
| Right stick | drive |
| Left stick | turn |
| R1 | arm down |
| R2 / L2 | arm mid |
| R2 / L1 | support out |
| D-pad up / down | trim arm rest position, 2 deg a press, resets on power cycle |
| D-pad left | siren lights |
| D-pad right | victory rainbow |
| Triangle / X | LED brightness up / down |

### `MiniBot_Telemetry/`

The competition firmware plus a Wi-Fi telemetry feed, for bench testing and
practice. Same driving code, so it behaves the same on the field.

Press **Circle** to raise an access point called `MiniBot`, press it again to
shut the radio off. Always off at boot, so a power cycle leaves it match-safe.

Needs the `huge_app` partition scheme, which `sketch.yaml` already pins.

`dashboard/index.html` opens on the laptop, not the robot. Join the `MiniBot`
network and open the file. It shows live controller input, arm position, battery
with history, reset reason, worst control-link gap, loop rate and per-servo
angles.

Keep the telemetry build off the robot during matches until the Wi-Fi has been
proven not to disturb the controller link.
