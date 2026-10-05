#!/usr/bin/env python3
"""
Checks of writer_robot/beacon_graph_nav.py (the beacon-graph navigation that
is ready but not yet used by the Executor). No ROS needed.

    python3 tools/test_beacon_graph.py
"""
import itertools
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import sim2d  # noqa: E402
from beacon_graph_demo import world_line_free  # noqa: E402
from writer_robot.beacon_graph_nav import BeaconGraph, BeaconPoint, GraphParams, INF, _seg_point_dist  # noqa: E402

results = []


def check(ok, text):
    results.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {text}')


def synthetic():
    print('== synthetic: two branches of the tree, 1 m apart')
    # EXIT(0) - 1 - 2 (east), EXIT - 3 - 4 (north then east): 2 and 4 are 1 m apart
    b = [BeaconPoint(0, 0, 0, 'EXIT'), BeaconPoint(1, 3, 0, parent=0), BeaconPoint(2, 6, 0, parent=1),
         BeaconPoint(3, 0, 1, parent=0), BeaconPoint(4, 6, 1, 'GAS', parent=3, target=(7.5, 1.0))]
    tree = BeaconGraph(b, GraphParams(link_radius=0))
    g = BeaconGraph(b, GraphParams(link_radius=1.5), line_free=lambda a, c: True)
    t, d, s = tree.dijkstra(2, 4), g.dijkstra(2, 4), g.astar(2, 4)
    check(t.ids == [2, 1, 0, 3, 4] and abs(t.length - 13.0) < 1e-9, f'tree only: 2 -> 4 goes back through the EXIT ({t.length:.1f} m)')
    check(d.ids == [2, 4] and abs(d.length - 1.0) < 1e-9, f'with a shortcut: 2 -> 4 directly ({d.length:.1f} m)')
    check(s.ids == d.ids, 'A* finds the same route as Dijkstra')
    blocked = BeaconGraph(b, GraphParams(link_radius=1.5), line_free=lambda a, c: False)
    check(blocked.dijkstra(2, 4).ids == [2, 1, 0, 3, 4], 'a wall between them (line_free False): no shortcut')
    haz = BeaconGraph(b, GraphParams(link_radius=1.5), line_free=lambda a, c: True, hazards=[(6.0, 0.5)])
    check(haz.dijkstra(2, 4).ids == [2, 1, 0, 3, 4] or haz.dijkstra(2, 4).cost > 1.0,
          'a hazard on the shortcut: removed or penalised')
    check(not BeaconGraph(b).dijkstra(2, 99).ok, 'unknown beacon: no route (no crash)')
    order, total = g.order_targets(0, [4, 2], end=0)
    check(sorted(order) == [2, 4], f'order_targets visits both targets: {order}, {total:.1f} m')


def mission(path):
    print(f'== {os.path.basename(path)}')
    m = json.load(open(path))
    world = sim2d.World(os.path.join(ROOT, 'worlds', m.get('world') or 'contaminated_zone.world'))
    free = world_line_free(world)
    tree = BeaconGraph.from_mission(m, params=GraphParams(link_radius=0))
    g = BeaconGraph.from_mission(m, params=GraphParams(link_radius=4.0), line_free=free)
    ids = sorted(g.nodes)
    pairs = list(itertools.combinations(ids, 2))
    if len(pairs) > 400:
        pairs = pairs[::len(pairs) // 400]
    ok_tree = all(abs(tree.dijkstra(a, b).length - tree.tree_path(a, b).length) < 1e-6 for a, b in pairs)
    check(ok_tree, f'tree only: Dijkstra = the tree route for {len(pairs)} pairs')
    same = all(abs(g.dijkstra(a, b).cost - g.astar(a, b).cost) < 1e-6 for a, b in pairs)
    check(same, 'A* cost = Dijkstra cost for every pair (admissible heuristic)')
    ed = sum(g.dijkstra(a, b).expanded for a, b in pairs)
    ea = sum(g.astar(a, b).expanded for a, b in pairs)
    check(ea <= ed, f'A* expands fewer nodes: {ea} vs {ed} (Dijkstra)')
    better = all(g.dijkstra(a, b).cost <= g.tree_path(a, b).cost + 1e-6 for a, b in pairs)
    check(better, 'graph route never costs more than the tree route')
    bad = [(i, j) for i, j, w, k in g.edges() if k == 'shortcut'
           and not free((g.nodes[i].x, g.nodes[i].y), (g.nodes[j].x, g.nodes[j].y))]
    check(not bad, f'every shortcut is wall-free ({sum(1 for e in g.edges() if e[3] == "shortcut")} shortcuts)')
    cross = [(i, j) for i, j, w, k in g.edges() if k == 'shortcut' and any(
        _seg_point_dist((g.nodes[i].x, g.nodes[i].y), (g.nodes[j].x, g.nodes[j].y), h) < 1.2
        and math.hypot(g.nodes[i].x - h[0], g.nodes[i].y - h[1]) >= 1.2
        and math.hypot(g.nodes[j].x - h[0], g.nodes[j].y - h[1]) >= 1.2 for h in g.hazards)]
    check(not cross, 'no shortcut crosses a hazard keep-out zone')
    root = [b.id for b in g.nodes.values() if b.kind == 'EXIT'][0]
    haz = [b.id for b in g.nodes.values() if b.kind not in ('EXIT', 'WAYPOINT')]
    order, total = g.order_targets(root, haz, end=root)
    torder, ttotal = tree.order_targets(root, haz, end=root)
    check(sorted(order) == sorted(haz) and total <= ttotal + 1e-6,
          f'all {len(haz)} hazards ordered: graph {total:.1f} m <= tree {ttotal:.1f} m')


def main():
    synthetic()
    for f in ('demo_big.json', 'demo_retreat_test.json'):
        mission(os.path.join(ROOT, 'missions', f))
    n = sum(results)
    print(f'\n{n}/{len(results)} checks passed' + (' - ALL PASSED' if n == len(results) else ''))
    return 0 if n == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
