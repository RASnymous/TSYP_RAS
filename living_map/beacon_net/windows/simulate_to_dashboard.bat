@echo off
rem Simulated beacon network (20x real time) -> gateway bridge -> dashboard.
rem Start run_dashboard.bat first. Extra options go to the simulator, e.g.:
rem   simulate_to_dashboard.bat --beacons 30 --rubble 2.5
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% ..\python\beaconnet\gateway_bridge.py --sim="--speed 20 %*" --server http://localhost:3000
pause
