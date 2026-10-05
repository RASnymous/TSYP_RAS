@echo off
rem The ONA for the GAFSA MINE: gateways at the portal and on the cable backbone underground,
rem the mine plan sent to the dashboard. Then simulate_mine.bat (or the robots: run_explorer.sh mine ona).
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona --config ona_config_gazebo_mine.json --udp 0.0.0.0:47100 --command-post http://localhost:3000 %*
pause
