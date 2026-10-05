@echo off
rem Zone simulator in the GAFSA MINE: the recorded mine mission as the three gateways hear it.
rem The Executor waits at the portal for a briefing (dispatch one on the dashboard), then goes
rem for the trapped miner first and every danger. Start the dashboard and run_ona_mine.bat first.
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona.zonesim --config ona_config_gazebo_mine.json --mission ..\writer_robot\missions\demo_mine.json --speed 2 %*
pause
