@echo off
rem Real LoRa gateway board on USB -> dashboard. Find the COM port in Device Manager
rem (Ports (COM & LPT)). Start run_dashboard.bat first.
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m pip install --quiet pyserial
set PORT=
set /p PORT=Gateway COM port (e.g. COM5): 
set KEY=demo
set /p KEY=Mission key, 32 hex characters (Enter = demo key): 
%PY% ..\python\beaconnet\gateway_bridge.py --serial %PORT% --key %KEY% --server http://localhost:3000
pause
