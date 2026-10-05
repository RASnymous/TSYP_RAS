@echo off
rem C (bptool.exe) versus Python cross-checks: frames, HMAC, JSON parser, digests, airtime
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% ..\python\tests\test_crosscheck.py
pause
