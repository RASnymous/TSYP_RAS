@echo off
rem The whole mission in the GAFSA MINE (2D simulator): the Writer explores with the mine sensors,
rem marks dangers, resources and searched areas; the Executor treats the trapped miner first.
cd /d "%~dp0.."
python tools\sim_mission.py --world worlds\gafsa_mine.world --png mission_mine.png
start "" mission_mine.png
pause
