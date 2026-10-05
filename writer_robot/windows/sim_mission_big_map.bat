@echo off
cd /d "%~dp0.."
python tools\sim_mission.py --png mission_big_map.png
start "" mission_big_map.png
pause
