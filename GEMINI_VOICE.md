# Gemini and ABR control

Start everything from the repository root with `.\start.bat`, or use the default VS Code build task (Ctrl+Shift+B) or the MiniBot F5 launch configuration. `python start.py` also works with the configured Python 3.12 interpreter. The launcher opens the panel, uses camera 1 and Bluetooth COM7, warms ABR and checks the saved Gemini key. It reuses an existing panel instead of opening another server. It never reinstalls dependencies. WSL stays alive while the server runs; Ctrl+C shuts the server down. The robot starts disabled and the browser still needs its normal microphone permission and Voice ON click. Override devices with `--camera N --port COMx` when needed.

The existing panel supports Windows microphone audio → local ABR ASR in WSL → Gemini text planning → validated actions. ABR TTS generates short spoken replies in parallel with action execution; movement never waits for synthesis or playback. Microphone capture pauses only during actual spoken output to avoid recognizing its own reply. STOP also aborts pending speech playback. Gemini replies are limited to 160 characters and prompted to use at most 12 words.

The Gemini checkbox selects flexible interpretation; unchecking it preserves the basic command parser. ABR spoken replies can be turned off separately. Only transcribed/typed message text is sent to Google. There is no camera access or obstacle avoidance in Gemini mode.

Credentials live outside this repository at `C:\Users\budhu\MiniBot\abr-private\gemini.key` and `license.key`. Replace a Gemini key by changing the private file; it is read on each request. Do not paste credentials into source code or the browser. Model selection defaults to `gemini-3.5-flash-lite`; `MINIBOT_GEMINI_MODEL` can override it on the server.

Examples:

- “Back up a little, then turn left for half a second.”
- “Turn right for one second, go back for three seconds, repeat twice.”
- “Show a blue snake pattern on all your lights.”
- “Make the lights breathe purple.”
- “Show a rainbow light show.”
- “Restore your status lights.”
- “What can you do?”
- “Stop.”

Movement requires the user to Enable first. Model plans allow 6 actions per cycle, 1–3 cycles, at most 12 expanded steps, and at most 10 seconds of movement overall. Each move lasts 0.25–5 seconds at existing manual speed. Indefinite repeats are unsupported. Exact local stop phrases bypass Gemini. STOP, Voice OFF, other panel controls, tab hiding and window blur cancel remaining steps. A failed command aborts the sequence. Connection and firmware manual timeouts remain active.

The firmware exposes light commands 10–16 and reports `lights` and `light_patterns` capabilities. These commands control both installed FastLED arrays. Effects: solid, blink, snake, chase, rainbow and breathe. Custom lights expire after 15 seconds; disabled always restores red. Blinking flashes for 2 seconds and then holds the color. Patterns run without blocking the motor loop. Servo accessory ports remain disabled.

Gemini proposes plans; logs distinguish the proposal, accepted movement requests, and stop commands. Software acceptance is not proof of physical movement. Test with wheels lifted before a floor run. Speech and cloud processing add latency, so use the panel STOP button for immediate intervention.
