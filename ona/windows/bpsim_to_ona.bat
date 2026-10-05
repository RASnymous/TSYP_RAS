@echo off
rem The beacon network simulator (C, the same code as the ESP32 beacons) with THREE gateways,
rem through the ONA's 2-of-3 vote, to the dashboard at http://localhost:3000. A 60-minute
rem mission at 10x: 6 minutes. Beacon 6 is destroyed at minute 25, an attacker sends forged frames.
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
"..\beacon_net\windows\bpsim.exe" --gateways 3 --speed 10 --quiet | %PY% -m ona --stdin --config ona_config_bpsim.json --command-post http://localhost:3000 %*
pause
