"""
Beacon-graph navigation for the Executor: the beacons are the points of a
graph, and routes between them are found with Dijkstra or A*.

STATUS: ready to use, NOT wired into the Executor yet. The Executor today
walks the beacon TREE (executor_core._tree_path: up to the common ancestor,
then down). This module lets you replace that later; the "How to plug it in"
section at the end says exactly where.

Why a graph and not only the tree
---------------------------------
The Writer's `next` links form a tree rooted at the EXIT: every beacon has one
way out. Two branches that pass close to each other (for example on both
sides of a doorway) are only connected through their common ancestor, which
can be a long detour. A graph adds SHORTCUT edges between beacons that are
close and see each other, so the shortest route can jump from one branch to
the other.

    nodes   every beacon, at its DROP point (where it lies on the floor)
    edges   1. tree edges: beacon -> its `next` (always kept, proven drivable:
               the Writer drove it)
            2. shortcut edges (optional): two beacons closer than
               `link_radius` whose straight line is free on an occupancy grid
               (a `line_free(a, b)` function you pass in, e.g. the Executor's
               own SLAM map)
    weight  edge length, x `shortcut_factor` for shortcuts (>= 1: a proven tree
            edge is preferred when both are about as long), + `hazard_penalty`
            if the edge passes within `hazard_margin` of a hazard; an edge
            that crosses a keep-out disc is removed

Algorithms (all return a Route: beacon ids, points, length, nodes expanded)
    dijkstra(a, b)       shortest route, explores in every direction
    astar(a, b)          same result, fewer nodes expanded (straight-line
                         distance to the goal as heuristic: admissible because
                         every edge weight >= its length)
    tree_path(a, b)      what the Executor does today (for comparison)
    shortest_from(a)     Dijkstra to every beacon (distances + parents)
    order_targets(start, targets, end)
                         visiting order for several hazards: nearest-neighbour
                         on graph distances, improved by 2-opt, returning to
                         `end` (the EXIT)

No ROS inside: the same code runs in the node, the 2D simulator and on
Windows. Try it on a mission file:
    python3 tools/beacon_graph_demo.py --mission missions/demo_big.json --png /tmp/graph.png
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

INF = float('inf')
Point = Tuple[float, float]


@dataclass
class BeaconPoint:
    id: int
    x: float                     # where the beacon lies (drop point)
    y: float
    kind: str = 'WAYPOINT'       # EXIT, WAYPOINT, RADIATION, THERMAL, GAS, VICTIM ...
    parent: Optional[int] = None  # LMB2 `next` (the way out)
    target: Optional[Point] = None   # hazards: the hazard itself (record position)


@dataclass
class Route:
    ids: List[int]
    points: List[Point]
    length: float                # metres along the beacons
    cost: float = INF            # what the search minimised (length + penalties)
    expanded: int = 0            # nodes taken off the priority queue (work of the search)
    method: str = ''

    @property
    def ok(self) -> bool:
        return bool(self.ids)


@dataclass
class GraphParams:
    link_radius: float = 4.0         # shortcut edges between beacons closer than this (m); 0 = tree only
    shortcut_factor: float = 1.15    # shortcut weight = length x this (tree edges are proven drivable)
    hazard_margin: float = 0.5       # edges passing within keepout + this get a penalty
    hazard_penalty: float = 3.0      # m added to such an edge
    keepout_radius: float = 1.2      # an edge crossing a hazard's keep-out disc is removed
    max_neighbours: int = 8          # shortcut edges per beacon (closest first)


def _seg_point_dist(a: Point, b: Point, p: Point) -> float:
    ax, ay = a
    bx, by = b
    ex, ey = bx - ax, by - ay
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * ex + (p[1] - ay) * ey) / L2))
    return math.hypot(ax + ex * t - p[0], ay + ey * t - p[1])


class BeaconGraph:
    """Undirected weighted graph of beacons."""

    def __init__(self, beacons: Iterable[BeaconPoint], params: Optional[GraphParams] = None,
                 line_free: Optional[Callable[[Point, Point], bool]] = None,
                 hazards: Sequence[Point] = ()):
        self.p = params or GraphParams()
        self.nodes: Dict[int, BeaconPoint] = {b.id: b for b in beacons}
        self.adj: Dict[int, Dict[int, float]] = {i: {} for i in self.nodes}
        self.kind_of_edge: Dict[Tuple[int, int], str] = {}
        self.hazards = list(hazards)
        self._build(line_free)

    # ------------------------------------------------------------ building
    @classmethod
    def from_executor(cls, core, params: Optional[GraphParams] = None, use_map: bool = True):
        """From a running ExecutorCore (after its tree is built): its beacon
        nodes (drop points), its hazards, and its own SLAM grid for shortcuts."""
        beacons = [BeaconPoint(n.id, n.drop[0], n.drop[1], n.kind, n.parent,
                               None if n.kind in ('WAYPOINT', 'EXIT') else (n.x, n.y))
                   for n in core.nodes.values() if n.drop is not None]
        hazards = [(n.x, n.y) for n in core.nodes.values() if n.kind not in ('WAYPOINT', 'EXIT')]
        hazards += [(h.x, h.y) for h in core.hazards]
        line_free = None
        if use_map and core.grid is not None:
            g = core.grid

            def line_free(a, b):         # noqa: E306 - no wall on the straight line (SLAM map)
                return core._line_free(g, g.cell(*a), g.cell(*b))
        return cls(beacons, params, line_free, hazards)

    @classmethod
    def from_mission(cls, mission: dict, key='demo', params: Optional[GraphParams] = None,
                     line_free=None):
        """From a mission file (verifies and decodes the 44-byte frames, then
        rebuilds the drop points exactly like the Executor does)."""
        from writer_robot.mission_log import decode_mission
        from writer_robot.executor_core import ExecutorCore
        records, _rejected, geo = decode_mission(mission, key)
        core = ExecutorCore(log=lambda *_: None, geo=geo)
        for r in records:
            core.records[int(r['beacon_id'])] = r
        core._build_tree()
        g = cls.from_executor(core, params, use_map=False)
        if line_free is not None:
            g = cls(g.nodes.values(), params, line_free, g.hazards)
        return g

    def _hazard_cost(self, a: Point, b: Point) -> float:
        """0 = fine, INF = crosses a keep-out disc, else a penalty."""
        cost = 0.0
        for h in self.hazards:
            d = _seg_point_dist(a, b, h)
            if d < self.p.keepout_radius:
                # a beacon INSIDE the zone (the hazard beacon itself) may still
                # connect: only reject if neither end is inside
                if math.hypot(a[0] - h[0], a[1] - h[1]) >= self.p.keepout_radius and \
                        math.hypot(b[0] - h[0], b[1] - h[1]) >= self.p.keepout_radius:
                    return INF
                cost += self.p.hazard_penalty
            elif d < self.p.keepout_radius + self.p.hazard_margin:
                cost += self.p.hazard_penalty
        return cost

    def _add_edge(self, i, j, w, kind):
        if i == j or w == INF:
            return
        if w < self.adj[i].get(j, INF):
            self.adj[i][j] = w
            self.adj[j][i] = w
            self.kind_of_edge[(min(i, j), max(i, j))] = kind

    def _build(self, line_free):
        # 1. tree edges (never removed: the Writer drove them)
        for b in self.nodes.values():
            if b.parent is not None and b.parent in self.nodes:
                p = self.nodes[b.parent]
                L = math.hypot(b.x - p.x, b.y - p.y)
                hc = self._hazard_cost((b.x, b.y), (p.x, p.y))
                self._add_edge(b.id, p.id, L + (0.0 if hc == INF else hc), 'tree')
        # 2. shortcuts
        if self.p.link_radius <= 0:
            return
        ids = sorted(self.nodes)
        for i in ids:
            a = self.nodes[i]
            near = []
            for j in ids:
                if j == i:
                    continue
                b = self.nodes[j]
                L = math.hypot(a.x - b.x, a.y - b.y)
                if L <= self.p.link_radius:
                    near.append((L, j))
            near.sort()
            for L, j in near[:self.p.max_neighbours]:
                if j in self.adj[i]:
                    continue
                b = self.nodes[j]
                if line_free is not None and not line_free((a.x, a.y), (b.x, b.y)):
                    continue
                if line_free is None:
                    continue                 # without a map, no shortcut can be trusted
                hc = self._hazard_cost((a.x, a.y), (b.x, b.y))
                self._add_edge(i, j, L * self.p.shortcut_factor + hc, 'shortcut')

    # ------------------------------------------------------------ helpers
    def edges(self):
        """(i, j, weight, kind) for every edge once."""
        for (i, j), kind in self.kind_of_edge.items():
            yield i, j, self.adj[i][j], kind

    def _route(self, ids, expanded, method) -> Route:
        if not ids:
            return Route([], [], INF, INF, expanded, method)
        pts = [(self.nodes[i].x, self.nodes[i].y) for i in ids]
        L = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(pts[:-1], pts[1:]))
        C = sum(self.adj[u][v] for u, v in zip(ids[:-1], ids[1:]))
        return Route(ids, pts, L, C, expanded, method)

    def nearest(self, x: float, y: float, line_free=None, max_dist: float = 6.0) -> Optional[int]:
        """Closest beacon to (x, y) (in plain sight if line_free is given)."""
        best = None
        for b in self.nodes.values():
            d = math.hypot(b.x - x, b.y - y)
            if d > max_dist or (best is not None and d >= best[0]):
                continue
            if line_free is not None and not line_free((x, y), (b.x, b.y)):
                continue
            best = (d, b.id)
        return None if best is None else best[1]

    # ------------------------------------------------------------ searches
    def shortest_from(self, src: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]]]:
        """Dijkstra from src to every beacon: (distance, parent) dictionaries."""
        dist = {src: 0.0}
        par: Dict[int, Optional[int]] = {src: None}
        pq = [(0.0, src)]
        done = set()
        while pq:
            d, u = heapq.heappop(pq)
            if u in done:
                continue
            done.add(u)
            for v, w in self.adj[u].items():
                nd = d + w
                if nd < dist.get(v, INF):
                    dist[v] = nd
                    par[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, par

    def dijkstra(self, src: int, dst: int) -> Route:
        if src not in self.nodes or dst not in self.nodes:
            return Route([], [], INF, INF, 0, 'dijkstra')
        dist = {src: 0.0}
        par = {src: None}
        pq = [(0.0, src)]
        done = set()
        while pq:
            d, u = heapq.heappop(pq)
            if u in done:
                continue
            done.add(u)
            if u == dst:
                break
            for v, w in self.adj[u].items():
                nd = d + w
                if nd < dist.get(v, INF):
                    dist[v] = nd
                    par[v] = u
                    heapq.heappush(pq, (nd, v))
        return self._route(self._unwind(par, dst), len(done), 'dijkstra')

    def astar(self, src: int, dst: int) -> Route:
        if src not in self.nodes or dst not in self.nodes:
            return Route([], [], INF, INF, 0, 'astar')
        gx, gy = self.nodes[dst].x, self.nodes[dst].y

        def h(i):
            n = self.nodes[i]
            return math.hypot(n.x - gx, n.y - gy)   # admissible: weights >= lengths
        g = {src: 0.0}
        par = {src: None}
        pq = [(h(src), 0.0, src)]
        closed = set()
        while pq:
            _f, gu, u = heapq.heappop(pq)
            if u in closed:
                continue
            closed.add(u)
            if u == dst:
                break
            for v, w in self.adj[u].items():
                ng = gu + w
                if ng < g.get(v, INF):
                    g[v] = ng
                    par[v] = u
                    heapq.heappush(pq, (ng + h(v), ng, v))
        return self._route(self._unwind(par, dst), len(closed), 'astar')

    def tree_path(self, src: int, dst: int) -> Route:
        """The Executor's current behaviour: up to the common ancestor, then down."""
        def chain(i):
            out, seen = [], set()
            while i is not None and i in self.nodes and i not in seen:
                out.append(i)
                seen.add(i)
                i = self.nodes[i].parent
            return out
        ca, cb = chain(src), chain(dst)
        sb = {v: k for k, v in enumerate(cb)}
        for k, v in enumerate(ca):
            if v in sb:
                ids = ca[:k + 1] + list(reversed(cb[:sb[v]]))
                pts = [(self.nodes[i].x, self.nodes[i].y) for i in ids]
                L = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(pts[:-1], pts[1:]))
                C = sum(self.adj[u].get(v, INF) for u, v in zip(ids[:-1], ids[1:]))
                return Route(ids, pts, L, C, 0, 'tree')
        return Route([], [], INF, INF, 0, 'tree')

    @staticmethod
    def _unwind(par, dst):
        if dst not in par:
            return []
        out = []
        u = dst
        while u is not None:
            out.append(u)
            u = par[u]
        return list(reversed(out))

    # ------------------------------------------------------------ several targets
    def order_targets(self, start: int, targets: Sequence[int], end: Optional[int] = None) -> Tuple[List[int], float]:
        """Order in which to visit `targets` from `start`, then go to `end`
        (e.g. the EXIT): nearest neighbour on graph distances + 2-opt.
        Returns (order, total length). Unreachable targets are left out."""
        pts = [start] + [t for t in targets if t != start]
        D = {a: self.shortest_from(a)[0] for a in set(pts) | ({end} if end is not None else set())}
        reach = [t for t in pts[1:] if D[start].get(t, INF) < INF]
        order, cur, left = [], start, set(reach)
        while left:
            nxt = min(left, key=lambda t: D[cur].get(t, INF))
            order.append(nxt)
            left.remove(nxt)
            cur = nxt

        def total(seq):
            s, c = 0.0, start
            for t in seq:
                s += D[c].get(t, INF)
                c = t
            if end is not None:
                s += D[c].get(end, INF)
            return s
        best = total(order)
        improved = True
        while improved and len(order) > 2:
            improved = False
            for i in range(len(order) - 1):
                for j in range(i + 1, len(order)):
                    cand = order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:]
                    c = total(cand)
                    if c < best - 1e-9:
                        order, best, improved = cand, c, True
        return order, best


# ----------------------------------------------------------------------------
# How to plug it in (later)
# ----------------------------------------------------------------------------
# In writer_robot/executor_core.py:
#
#   1. at the top:   from writer_robot.beacon_graph_nav import BeaconGraph, GraphParams
#
#   2. at the end of _start_mission() (the tree and the keep-outs exist):
#          self.graph = BeaconGraph.from_executor(self, GraphParams(keepout_radius=self.p.keepout_radius))
#
#   3. wherever a beacon route is made from the tree, i.e. every call of
#          ids = self._tree_path(self.current, t)
#      (in _next_target / _set_route, and the RETURN to the exit), use instead
#          r = self.graph.astar(self.current, t)          # or .dijkstra(...)
#          ids = r.ids if r.ok else self._tree_path(self.current, t)
#      and, to choose the next hazard, replace _tree_dist(a, b) by
#          self.graph.shortest_from(a)[0].get(b, INF)
#      or plan the whole visiting order once with
#          order, length = self.graph.order_targets(self.current, self.targets, end=self.root)
#
#   4. rebuild self.graph when the map has grown a lot (shortcuts use the
#      Executor's own SLAM map): e.g. every 30 s in step(), or when a leg fails.
#
#   5. keep the tree as the fallback: a shortcut is only as good as the map
#      it was checked on, while every tree edge was driven by the Writer.
#      If a shortcut leg fails (_leg_failed), remove that edge
#          self.graph.adj[a].pop(b, None); self.graph.adj[b].pop(a, None)
#      and plan again.
#
# Test it offline first:
#   python3 tools/beacon_graph_demo.py --mission missions/demo_big.json --png /tmp/graph.png
#   python3 tools/test_beacon_graph.py
