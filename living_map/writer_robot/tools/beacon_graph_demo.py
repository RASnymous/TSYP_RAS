#!/usr/bin/env python3
"""
Try the beacon-graph navigation (writer_robot/beacon_graph_nav.py) on a
mission file, without ROS and without changing the Executor.

    python3 tools/beacon_graph_demo.py                                 (demo_big mission)
    python3 tools/beacon_graph_demo.py --mission ~/writer_robot_ws/missions/latest.json --png /tmp/graph.png
    python3 tools/beacon_graph_demo.py --link-radius 0                 (tree only)

It prints, for every hazard beacon, the route from the EXIT along the tree
(what the Executor does today) and with Dijkstra and A* on the graph (tree +
shortcuts checked against the world's walls), and the visiting order of all
hazards. With --png it draws the graph.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import sim2d  # noqa: E402
from writer_robot.beacon_graph_nav import BeaconGraph, GraphParams, INF  # noqa: E402

KIND_COLOR = {'EXIT': '#18a558', 'WAYPOINT': '#1e7ae0', 'RADIATION': '#d0d', 'THERMAL': '#e22',
              'GAS': '#eb0', 'VICTIM': '#0cc'}


def world_line_free(world, clearance=0.30):
    """Straight line a-b drivable in the world: every point of it at least
    `clearance` from a wall (the first/last 0.3 m excepted: beacons lie near
    walls). Hazard blocks are left to the graph's keep-out rule."""
    hz = {h[6] for h in world.hazards}
    segs = []
    for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
        if name not in hz and z1 > 0.2:
            segs.extend(sim2d.rect_segments(cx, cy, sx, sy, yw))
    S = np.array(segs)

    def free(a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(L / 0.08))
        t = np.linspace(0.0, 1.0, n)
        keep = (t * L >= 0.3) & ((1 - t) * L >= 0.3)
        if not keep.any():
            return True
        px = a[0] + (b[0] - a[0]) * t[keep]
        py = a[1] + (b[1] - a[1]) * t[keep]
        ax, ay, bx, by = S[:, 0][None, :], S[:, 1][None, :], S[:, 2][None, :], S[:, 3][None, :]
        ex, ey = bx - ax, by - ay
        L2 = np.maximum(ex * ex + ey * ey, 1e-12)
        u = np.clip(((px[:, None] - ax) * ex + (py[:, None] - ay) * ey) / L2, 0.0, 1.0)
        d = np.hypot(ax + ex * u - px[:, None], ay + ey * u - py[:, None])
        return bool(d.min() >= clearance)
    return free


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mission', default=os.path.join(ROOT, 'missions', 'demo_big.json'))
    ap.add_argument('--world', help='world file (default: the one named in the mission)')
    ap.add_argument('--key', default='demo')
    ap.add_argument('--link-radius', type=float, default=4.0, help='shortcut edges up to this length (0 = tree only)')
    ap.add_argument('--png', help='draw the graph into this picture')
    a = ap.parse_args()

    m = json.load(open(os.path.expanduser(a.mission)))
    wpath = a.world or os.path.join(ROOT, 'worlds', m.get('world') or 'contaminated_zone.world')
    world = sim2d.World(wpath)
    free = world_line_free(world)
    tree = BeaconGraph.from_mission(m, a.key, GraphParams(link_radius=0))
    graph = BeaconGraph.from_mission(m, a.key, GraphParams(link_radius=a.link_radius), line_free=free)
    n_short = sum(1 for e in graph.edges() if e[3] == 'shortcut')
    exits = [b.id for b in graph.nodes.values() if b.kind == 'EXIT']
    root = exits[0] if exits else min(graph.nodes)
    hazards = [b.id for b in graph.nodes.values() if b.kind not in ('EXIT', 'WAYPOINT')]
    print(f'mission : {a.mission}')
    print(f'world   : {os.path.basename(wpath)}')
    print(f'graph   : {len(graph.nodes)} beacons, {len(tree.kind_of_edge)} tree edges, '
          f'{n_short} shortcut edges (<= {a.link_radius} m, wall-free)')
    print()
    print(f'  route EXIT #{root} -> hazard    tree (today)      Dijkstra                A*')
    for h in sorted(hazards):
        t = graph.tree_path(root, h)
        d = graph.dijkstra(root, h)
        s = graph.astar(root, h)
        kind = graph.nodes[h].kind
        print(f'  {kind:>9} #{h:<4}           {t.length:6.1f} m {len(t.ids):3d} b   '
              f'{d.length:6.1f} m {len(d.ids):3d} b {d.expanded:3d} exp   '
              f'{s.length:6.1f} m {len(s.ids):3d} b {s.expanded:3d} exp')
    order, total = graph.order_targets(root, hazards, end=root)
    torder, ttotal = tree.order_targets(root, hazards, end=root)
    print()
    print(f'  visit all hazards and return (cost, m):  tree {ttotal:.1f}  ({" -> ".join(f"#{i}" for i in torder)})')
    print(f'                                           graph {total:.1f}  ({" -> ".join(f"#{i}" for i in order)})')

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MPoly
        fig, ax = plt.subplots(figsize=(12, 8.5))
        for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
            col = '#888' if not any(name == h[6] for h in world.hazards) else '#f0a'
            ax.add_patch(MPoly(sim2d.rect_corners(cx, cy, sx, sy, yw), closed=True, fc=col, ec='none', alpha=0.8))
        for i, j, w, kind in graph.edges():
            A, B = graph.nodes[i], graph.nodes[j]
            if kind == 'tree':
                ax.plot([A.x, B.x], [A.y, B.y], '-', color='#1e7ae0', lw=1.6, zorder=2)
            else:
                ax.plot([A.x, B.x], [A.y, B.y], '--', color='#999', lw=0.8, zorder=1)
        if hazards:
            far = max(hazards, key=lambda h: graph.tree_path(root, h).length)
            r1, r2 = graph.tree_path(root, far), graph.astar(root, far)
            ax.plot([p[0] for p in r1.points], [p[1] for p in r1.points], '-', color='#f80', lw=5, alpha=0.35,
                    label=f'tree route to #{far}: {r1.length:.1f} m')
            ax.plot([p[0] for p in r2.points], [p[1] for p in r2.points], '-', color='#0a0', lw=3, alpha=0.7,
                    label=f'A* on the graph to #{far}: {r2.length:.1f} m')
        for b in graph.nodes.values():
            ax.plot(b.x, b.y, 'o', color=KIND_COLOR.get(b.kind, '#555'), ms=7 if b.kind != 'WAYPOINT' else 4, zorder=3)
            if b.kind != 'WAYPOINT':
                ax.annotate(f'{b.kind} #{b.id}', (b.x, b.y), xytext=(4, 4), textcoords='offset points', fontsize=7)
        ax.plot([], [], '-', color='#1e7ae0', label='tree edge (LMB2 next)')
        ax.plot([], [], '--', color='#999', label='shortcut edge (wall-free, <= %.1f m)' % a.link_radius)
        ax.set_aspect('equal')
        ax.legend(loc='lower right', fontsize=8)
        ax.set_title(f'Beacon graph: {len(graph.nodes)} beacons, {n_short} shortcuts')
        fig.tight_layout()
        fig.savefig(a.png, dpi=110)
        print(f'\npicture: {a.png}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
