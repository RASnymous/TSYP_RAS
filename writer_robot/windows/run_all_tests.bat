@echo off
cd /d "%~dp0.."
python tools\run_tests.py
python tools\run_mission_tests.py
python tools\test_nodes.py
python tools\test_beacon_graph.py
python tools\test_ona_link.py
pause
