@echo off
rem Full simulated mission as fast as possible, then the report and the 13 checks.
rem Options: --beacons 30 --minutes 90 --rubble 2.5 --sf 7 --kill 6@25 --seed 3 --help
cd /d "%~dp0"
bpsim.exe %* > nul
pause
