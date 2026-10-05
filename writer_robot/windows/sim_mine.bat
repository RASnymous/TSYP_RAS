@echo off
rem Only the Writer in the GAFSA MINE (2D simulator), with its checks.
cd /d "%~dp0.."
python tools\sim2d.py --world worlds\gafsa_mine.world --png mine_run.png
start "" mine_run.png
pause
