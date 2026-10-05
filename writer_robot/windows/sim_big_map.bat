@echo off
cd /d "%~dp0.."
python tools\sim2d.py --png big_map_run.png
start "" big_map_run.png
pause
