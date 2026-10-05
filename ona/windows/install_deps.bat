@echo off
rem One time: the Python packages the ONA needs (numpy; pyserial for real gateway boards).
setlocal
set PY=python
where py >nul 2>nul && set PY=py -3
%PY% -m pip install numpy pyserial
pause
