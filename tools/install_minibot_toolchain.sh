#!/usr/bin/env bash
# MiniBot / LoR Core V3 toolchain installer - macOS / Linux
# Run:  bash install_minibot_toolchain.sh
# Installs into the SAME folders Arduino IDE uses, so everything shows up in the IDE.

set -euo pipefail
ESP32SERVO_VERSION="3.0.7"   # pinned by Lord of Robots - do not bump

echo "=== MiniBot toolchain installer ==="

echo
echo "[1/5] Arduino IDE..."
if [[ "$OSTYPE" == darwin* ]] && command -v brew >/dev/null 2>&1; then
  brew install --cask arduino-ide || echo "  IDE may already be installed - continuing."
else
  echo "  Install the IDE by hand if you want the GUI: https://www.arduino.cc/en/software"
fi

echo
echo "[2/5] arduino-cli..."
CLI_DIR="$HOME/.local/bin"
mkdir -p "$CLI_DIR"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
  | BINDIR="$CLI_DIR" sh
CLI="$CLI_DIR/arduino-cli"

echo
echo "[3/5] Board packages (both are required)..."
"$CLI" config init --overwrite >/dev/null
"$CLI" config add board_manager.additional_urls \
  "https://dl.espressif.com/dl/package_esp32_index.json" \
  "https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
"$CLI" core update-index
"$CLI" core install esp32:esp32
"$CLI" core install esp32-bluepad32:esp32

echo
echo "[4/5] Libraries..."
"$CLI" lib install "ESP32Servo@${ESP32SERVO_VERSION}"
"$CLI" lib install "FastLED"

echo
echo "[5/5] Test compile..."
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INO="$REPO/MiniBot_Competition1.ino"
if [[ -f "$INO" ]]; then
  BUILD="$(mktemp -d)/MiniBot_Competition1"
  mkdir -p "$BUILD"
  cp "$INO" "$BUILD/"
  "$CLI" compile --fqbn esp32-bluepad32:esp32:esp32 "$BUILD"
else
  echo "  Sketch not found next to this script - skipping compile."
fi

echo
echo "=== Done ==="
echo "Open Arduino IDE and select: Tools > Board > esp32_bluepad32 > ESP32 Dev Module"
echo "Keep ESP32Servo at ${ESP32SERVO_VERSION} - decline IDE update prompts for it."
