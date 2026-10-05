#!/usr/bin/env python3
"""
run_mission_tests.py - the whole Living Map mission (Writer drops beacons,
Executor follows them) in the 2D simulator, on every map, with several random
seeds, a camera that sees hazards later, and an Executor whose map is not
aligned with the Writer's. One line per run. Linux and Windows.

    python3 tools/run_mission_tests.py            # reports go to ./test_reports/
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.getcwd(), 'test_reports')
W = lambda f: os.path.join(ROOT, 'worlds', f)  # noqa: E731

RUNS = [(f'mission_big_seed{s}', ['--seed', str(s)]) for s in range(1, 5)]
RUNS += [(f'mission_big_late_camera_seed{s}', ['--seed', str(s), '--min-area', '1500']) for s in (1, 2)]
RUNS += [('mission_big_radio_joins_mid_cycle', ['--seed', '1', '--radio-offset', '25']),
         ('mission_big_misaligned_a', ['--seed', '2', '--frame-error=0.3,-0.3,2']),
         ('mission_big_misaligned_b', ['--seed', '3', '--frame-error=-0.4,0.3,-3'])]
RUNS += [(f'mission_retreat_test_seed{s}', ['--world', W('retreat_test.world'), '--seed', str(s)]) for s in (1, 2)]
RUNS += [('mission_old_small_world', ['--world', W('contaminated_zone_small.world'), '--seed', '1'])]
# the ready-made demo mission with what can go wrong in Gazebo
D = ['--mission', os.path.join(ROOT, 'missions', 'demo_big.json')]
RUNS += [('demo_camera_range_25pct_off', D + ['--depth-scale', '1.25', '--bearing-bias', '5', '--wall-grow', '1']),
         ('demo_camera_range_25pct_short', D + ['--depth-scale', '0.75', '--bearing-bias', '-5', '--turn-slip', '0.7']),
         ('demo_beacon_heard_late', D + ['--late-records', '5:25,12:40']),
         ('demo_door_half_blocked', D + ['--extra-box=10.2,-3.6,0.6,0.2']),
         ('demo_door_blocked', D + ['--extra-box=10.5,-3.6,1.2,0.2', '--expect-skip', '13'])]
# Command Post briefings (dashboard -> ONA -> 3 gateways -> ona_link_node -> Executor)
RUNS += [('demo_briefing_in_order', D + ['--wait-briefing', '30', '--briefing', '5:25,11,20']),
         ('demo_briefing_retarget_stay', D + ['--briefing', '40:20,30:stay', '--briefing', '150:13']),
         ('demo_briefing_abort', D + ['--briefing', '45:abort']),
         ('demo_briefing_record_heard_late', D + ['--wait-briefing', '30', '--briefing', '3:13,25',
                                                  '--late-records', '25:30'])]
# the Gafsa mine: whole missions (victims first, roofs propped, resources and searched areas marked),
# the ready-made mine mission with a briefing that samples the rich phosphate seam, and its camera off
M = ['--world', W('gafsa_mine.world')]
DM = M + ['--mission', os.path.join(ROOT, 'missions', 'demo_mine.json')]
RUNS += [(f'mine_seed{s}', M + ['--seed', str(s)]) for s in (1, 2)]
RUNS += [('mine_demo_briefing_sample_then_victim', DM + ['--wait-briefing', '30', '--briefing', '5:51,53']),
         # range error of the mine sensors (thermal camera + depth), both ways, noisy SLAM walls; a gallery is
         # 2.8 m wide, so a 25 % error at 3 m already puts a fire against the wall inside the rock
         ('mine_demo_sensors_15pct_far', DM + ['--depth-scale', '1.15', '--bearing-bias', '4', '--wall-grow', '1']),
         ('mine_demo_sensors_15pct_short', DM + ['--depth-scale', '0.85', '--bearing-bias', '-4', '--wall-grow', '1'])]


def main():
    os.makedirs(OUT, exist_ok=True)
    npass, t0 = 0, time.time()
    for name, args in RUNS:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'sim_mission.py'), '--quiet'] + args,
                           capture_output=True, text=True)
        with open(os.path.join(OUT, name + '.txt'), 'w') as f:
            f.write(r.stdout + r.stderr)
        ok = r.returncode == 0
        npass += ok
        info = next((ln.split(':', 1)[1].strip() for ln in r.stdout.splitlines() if ln.startswith(' executor ')), '?')
        print(f'{name:34s} {"PASS" if ok else "FAIL"}  executor {info}', flush=True)
    print(f'passed {npass}/{len(RUNS)} in {time.time() - t0:.0f} s   (reports in {OUT})')
    return 0 if npass == len(RUNS) else 1


if __name__ == '__main__':
    sys.exit(main())
