@echo off
cd /d "%~dp0.."
python tools\sim_mission.py --world worlds\retreat_test.world --png mission_retreat_test.png
start "" mission_retreat_test.png
pause
