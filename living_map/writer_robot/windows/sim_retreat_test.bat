@echo off
cd /d "%~dp0.."
python tools\sim2d.py --world worlds\retreat_test.world --expect-turn-back --png retreat_test_run.png
start "" retreat_test_run.png
pause
