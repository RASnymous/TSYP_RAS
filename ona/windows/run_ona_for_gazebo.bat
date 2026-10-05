@echo off
rem The ONA on Windows for the Gazebo robots in the VM (big map). In the VM, start the robots with
rem   ONA_HOST=10.0.2.2 ~/writer_robot_ws/run_executor.sh demo headless ona
rem Windows may ask once to let Python through the firewall: allow it (private network).
rem Small map: run_ona_for_gazebo.bat --config ona_config_gazebo_retreat.json
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m ona --config ona_config_gazebo_big.json --udp 0.0.0.0:47100 --command-post http://localhost:3000 %*
pause
