#!/usr/bin/env python3
"""
run_tests.py - runs the explorer in the 2D simulator on every world, with
several random seeds and camera ranges, and prints one line per run.
Works on Linux and Windows (python + numpy).

    python3 tools/run_tests.py            # reports go to ./test_reports/
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.getcwd(), 'test_reports')

RUNS = [(f'big_seed{s}', ['--seed', str(s)]) for s in range(1, 7)]
RUNS += [(f'big_late_camera_seed{s}', ['--seed', str(s), '--min-area', '1500']) for s in range(1, 4)]
RUNS += [(f'big_very_late_camera_seed{s}', ['--seed', str(s), '--min-area', '3000']) for s in range(1, 4)]
RUNS += [('big_lidar_self_hits', ['--self-hits'])]
RUNS += [(f'retreat_test_seed{s}', ['--world', os.path.join(ROOT, 'worlds', 'retreat_test.world'),
                                    '--expect-turn-back', '--seed', str(s)]) for s in range(1, 4)]
RUNS += [(f'old_small_world_seed{s}', ['--world', os.path.join(ROOT, 'worlds', 'contaminated_zone_small.world'),
                                       '--seed', str(s)]) for s in range(1, 3)]
# the Gafsa mine: mine sensor suite, resources, victim search
RUNS += [(f'gafsa_mine_seed{s}', ['--world', os.path.join(ROOT, 'worlds', 'gafsa_mine.world'), '--seed', str(s)])
         for s in range(1, 4)]


def main():
    os.makedirs(OUT, exist_ok=True)
    npass = 0
    t0 = time.time()
    for name, args in RUNS:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'sim2d.py'), '--quiet'] + args,
                           capture_output=True, text=True)
        with open(os.path.join(OUT, name + '.txt'), 'w') as f:
            f.write(r.stdout + r.stderr)
        ok = r.returncode == 0
        npass += ok
        simt = next((ln.split(':', 1)[1].strip() for ln in r.stdout.splitlines() if 'simulated time' in ln), '?')
        print(f'{name:32s} {"PASS" if ok else "FAIL"}  {simt}', flush=True)
    print(f'passed {npass}/{len(RUNS)} in {time.time() - t0:.0f} s   (reports in {OUT})')
    return 0 if npass == len(RUNS) else 1


if __name__ == '__main__':
    sys.exit(main())
