@echo off
rem Dashboard + the GAFSA MINE mission replayed (no robot, no ONA needed): the mine plan, the victim
rem search area by area, resources, then the Executor waiting at the portal for a briefing.
cd /d "%~dp0..\..\command_post"
where node >nul 2>nul || (echo Node.js is missing: install the LTS version from https://nodejs.org then run this again. & pause & exit /b 1)
if not exist node_modules (echo Installing dashboard packages, one time only... & call npm install)
echo Gafsa mine demo: open http://localhost:3000   (Ctrl+C here to stop)
call npm run demo:mine
pause
