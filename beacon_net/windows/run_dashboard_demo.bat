@echo off
rem Dashboard + a scripted fake mission (no robot, no radio needed)
cd /d "%~dp0..\..\command_post"
where node >nul 2>nul || (echo Node.js is missing: install the LTS version from https://nodejs.org then run this again. & pause & exit /b 1)
if not exist node_modules (echo Installing dashboard packages, one time only... & call npm install)
echo Dashboard demo: open http://localhost:3000   (Ctrl+C here to stop)
call npm run demo
pause
