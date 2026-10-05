@echo off
rem Beacon-graph navigation (Dijkstra / A*) on the big demo mission, with a picture
cd /d "%~dp0.."
python tools\beacon_graph_demo.py --png beacon_graph.png
start "" beacon_graph.png
pause
