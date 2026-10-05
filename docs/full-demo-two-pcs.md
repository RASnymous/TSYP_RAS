# Full demo on two PCs (Writer live on the dashboard, then the Executor follows its beacons)

PC 1 = Ubuntu (robots, Gazebo, ONA). PC 2 = Windows (Command Post dashboard). Same Wi-Fi / router. All traffic goes PC 1 → PC 2 on port 3000 (the ONA posts and polls), so only PC 2 must accept incoming connections.

## A. Dashboard (PC 2)
1. Double-click `C:\tsyp\living_map\beacon_net\windows\run_dashboard.bat`; keep the window open. Firewall prompt for Node.js: tick Private networks, Allow.
2. Note the line `On LAN : http://192.168.x.x:3000   <- robot posts here`.
3. Browser on PC 2: http://localhost:3000 (empty map).

## B. Link PC 1 to it (Ubuntu terminal 1)
4. `bash ~/writer_robot_ws/stop_demo.sh`
5. `export COMMAND_POST=http://192.168.x.x:3000` (only for this terminal)
6. `curl $COMMAND_POST/api/health` must print `{"ok":true,...}`

## C. Writer
7. `~/writer_robot_ws/run_explorer.sh headless gpu ona` (add `retreat` for the small map). `gpu` = the 4 Oct libEGL fix.
8. Terminal 2: `tail -f /tmp/explorer.log`. Dashboard: the Writer moves, pins appear ✓3/3.
9. After DONE / EXPLORATION COMPLETE: Ctrl-C in terminal 1 (mission saved to `~/writer_robot_ws/missions/latest.json`). Do not restart the dashboard.

## D. Executor (same terminal 1)
10. `export WAIT_BRIEFING=300` (5 simulated minutes to dispatch instead of 60 s)
11. `~/writer_robot_ws/run_executor.sh headless gpu ona` (no `demo`: it loads the Writer's beacons)
12. Terminal 2: `tail -f /tmp/executor.log`, wait for "waiting up to 300 s for a briefing".
13. Dashboard: Select on map → click targets in order → Dispatch mission → "✓ acknowledged".
14. Watch: GOTO, approaching, TREATED, RETURN, MISSION COMPLETE. Optional: Call the Executor back.
15. End: Ctrl-C, `bash ~/writer_robot_ws/stop_demo.sh`.

## If it goes wrong
- curl fails: Windows firewall (Allow an app → Node.js → Private + Public), or not the same network; Ubuntu VM: NAT works, else Bridged adapter.
- Robots run but the dashboard stays empty: `tail /tmp/ona.log` on PC 1 (wrong address).
- `waiting for /scan` forever: keep `gpu`, drop `headless`.
- Executor started without the briefing: rerun step 11. Writer cut short: `run_executor.sh demo headless gpu ona`.
