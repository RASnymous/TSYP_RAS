@echo off
rem Living Map - Outside Network Area. Listens for the gateways (simulators and the VM robots
rem send UDP to port 47100) and posts to the Command Post at http://localhost:3000.
rem Extra options go through, e.g.:  run_ona.bat --lte-outage 60-150
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona --udp 0.0.0.0:47100 --command-post http://localhost:3000 %*
pause
