@echo off
rem Try the console's screens with a pretend device: no Jetson, no ssh, nothing is started.
cd /d "%~dp0"
start "APNVI fake helper" /min python ..\jetson\helper.py --fake
start "APNVI console (fake)" /min python hub.py --fake
timeout /t 2 /nobreak >nul
start "" http://localhost:8100
