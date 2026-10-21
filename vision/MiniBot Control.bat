@echo off
REM Double-click this to open the MiniBot control page.
REM
REM When the ESP-NOW dongle is plugged in, put its COM port on the next line
REM (for example  set PORT=COM5  ). Leave it blank to run vision only, which
REM sends nothing to the robot and is safe for testing.
set PORT=

set PY=C:\Users\ebene\AppData\Local\Programs\Python\Python312\python.exe
set APP=%~dp0web_ui.py

start "" http://localhost:8000

if "%PORT%"=="" (
  "%PY%" "%APP%"
) else (
  "%PY%" "%APP%" --port %PORT%
)

echo.
echo The control page has stopped. Press any key to close.
pause >nul
