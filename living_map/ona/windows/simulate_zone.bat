@echo off
rem Zone simulator: 3 gateways hear a mission in the built-in building and send it to the ONA.
rem The Executor waits at the exit for a briefing from the dashboard (120 s of mission time =
rem 60 s here at speed 2), then takes every hazard. Start run_ona.bat and the dashboard first.
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona.zonesim --speed 2 %*
pause
