@echo off
rem Living Map Command Post dashboard -> http://localhost:3000
cd /d "%~dp0..\..\command_post"
where node >nul 2>nul || (echo Node.js is missing: install the LTS version from https://nodejs.org then run this again. & pause & exit /b 1)
if not exist node_modules (echo Installing dashboard packages, one time only... & call npm install)
echo.
echo Dashboard: open http://localhost:3000   (Ctrl+C here to stop)
echo The Ubuntu VM reaches it at http://10.0.2.2:3000
echo.
call npm start
pause
