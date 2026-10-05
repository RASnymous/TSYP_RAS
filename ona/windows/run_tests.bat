@echo off
rem All ONA tests (about 2 minutes). Every block must end with "checks passed" and no FAIL.
setlocal
cd /d "%~dp0\.."
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% tests\test_ona.py
%PY% tests\test_zone.py
%PY% tests\test_vs_beaconnet.py
pause
