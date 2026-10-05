@echo off
rem Zone simulator with faults for the ONA to catch: gw3 lies, gw2 forges a victim at 60 s,
rem beacon 6 is crushed at 90 s, the robot SLAM slips 2.9 m at 150 s.
rem Start run_ona.bat first (add --lte-outage 120-180 to it to see the satellite backup).
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona.zonesim --speed 2 --liar gw3 --forge gw2@60 --kill 6@90 --slam-slip 150:2.5,-1.5 %*
pause
