@echo off
rem Remote link: the front door and its fixed ngrok address. A Startup shortcut to
rem this starts both at login. The ngrok address is in remote_link_domain.txt (one line, e.g.
rem something.ngrok-free.app; not committed). ngrok's login: ngrok config add-authtoken (once).
set HERE=%~dp0
set /p DOMAIN=<"%HERE%remote_link_domain.txt"
start "APNVI front door" /min python "%HERE%front_door.py"
start "APNVI ngrok" /min C:\APNVI\tools\ngrok\ngrok.exe http 8095 --url=https://%DOMAIN%
