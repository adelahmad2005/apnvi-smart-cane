@echo off
rem APNVI console. Put a shortcut to this file on the desktop.
rem It starts the console on this PC and opens it in the browser. The console's Get ready
rem button then starts the PC's servers, waits for the Jetson and starts the device.
cd /d "%~dp0"
start "APNVI console" /min python hub.py
timeout /t 2 /nobreak >nul
start "" http://localhost:8100
