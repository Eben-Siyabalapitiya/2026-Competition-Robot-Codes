# MiniBot / LoR Core V3 toolchain installer - Windows
# Run in PowerShell:  powershell -ExecutionPolicy Bypass -File install_minibot_toolchain.ps1
# Installs into the SAME folders Arduino IDE uses, so everything shows up in the IDE.

$ErrorActionPreference = "Stop"
$ESP32SERVO_VERSION = "3.0.7"   # pinned by Lord of Robots - do not bump

Write-Host "=== MiniBot toolchain installer ===" -ForegroundColor Cyan

# 1. Arduino IDE (skipped if already present)
Write-Host "`n[1/5] Arduino IDE..." -ForegroundColor Cyan
if (Get-Command winget -ErrorAction SilentlyContinue) {
    winget install -e --id ArduinoSA.IDE.stable --accept-source-agreements --accept-package-agreements
    if ($LASTEXITCODE -ne 0) { Write-Host "  IDE may already be installed - continuing." -ForegroundColor Yellow }
} else {
    Write-Host "  winget not available. Install the IDE by hand: https://www.arduino.cc/en/software" -ForegroundColor Yellow
}

# 2. arduino-cli (does the board/library installs)
Write-Host "`n[2/5] arduino-cli..." -ForegroundColor Cyan
$CliDir = Join-Path $env:LOCALAPPDATA "ArduinoCLI"
New-Item -ItemType Directory -Force -Path $CliDir | Out-Null
$Zip = Join-Path $env:TEMP "arduino-cli.zip"
Invoke-WebRequest -Uri "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip" -OutFile $Zip
Expand-Archive -Path $Zip -DestinationPath $CliDir -Force
$Cli = Join-Path $CliDir "arduino-cli.exe"

# 3. Board packages - BOTH are required
Write-Host "`n[3/5] Board packages..." -ForegroundColor Cyan
& $Cli config init --overwrite | Out-Null
& $Cli config add board_manager.additional_urls `
    "https://dl.espressif.com/dl/package_esp32_index.json" `
    "https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
& $Cli core update-index
& $Cli core install esp32:esp32
& $Cli core install esp32-bluepad32:esp32

# 4. Libraries
Write-Host "`n[4/5] Libraries..." -ForegroundColor Cyan
& $Cli lib install "ESP32Servo@$ESP32SERVO_VERSION"
& $Cli lib install "FastLED"

# 5. Compile the competition sketch
Write-Host "`n[5/5] Test compile..." -ForegroundColor Cyan
$Repo = Split-Path -Parent $PSScriptRoot
$Ino = Join-Path $Repo "MiniBot_Competition1.ino"
if (Test-Path $Ino) {
    $Build = Join-Path $env:TEMP "MiniBot_Competition1"
    New-Item -ItemType Directory -Force -Path $Build | Out-Null
    Copy-Item $Ino $Build -Force
    & $Cli compile --fqbn esp32-bluepad32:esp32:esp32 $Build
} else {
    Write-Host "  Sketch not found next to this script - skipping compile." -ForegroundColor Yellow
}

Write-Host "`n=== Done ===" -ForegroundColor Green
Write-Host "Open Arduino IDE and select: Tools > Board > esp32_bluepad32 > ESP32 Dev Module"
Write-Host "Keep ESP32Servo at $ESP32SERVO_VERSION - decline IDE update prompts for it."
