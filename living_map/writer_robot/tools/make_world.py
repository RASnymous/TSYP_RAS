#!/usr/bin/env python3
"""
make_world.py - generates the Gazebo worlds of the Writer robot:

    worlds/contaminated_zone.world   the big rescue map (default world)
    worlds/retreat_test.world        a small map to watch the "turn back" behaviour

    python3 tools/make_world.py            # rewrites both worlds

The layouts are written as data (wall segments with door gaps, rubble, hazard
blocks) so they are easy to read and change, and so the 2D test simulator
(tools/sim2d.py) uses exactly the same geometry as Gazebo.

BIG MAP - 20 m x 14 m, robot starts at (0, 0) facing +x:

  y=7 +-----------+-----------+-----------+----------------------+
      | Room NW   | Room N1   | Room N2   |  NE pocket   [GAS]   |
      |  rubble   |  rubble   |           |                      |
 y=3.6+---door----+--door-----+------door-+                      |
      |       north corridor (1.8 m wide)   |  ===============   |  wall y=2.5, gap at east end
      |              [RADIATION]            |                      |
 y=1.6|           +-------------------+ east|                      |
      |   HALL    | ISLAND  |  [FIRE] | cor-  door (x=13, y=0)     |
      |  start    |  room   |divider  | ridor |                      |
      |  (0,0)    +--door---+---------+     |   ===============  |  wall y=-2.5, gap at west end
      |        south corridor               |                      |
 y=-3.6+---door---+--door-----+-----door--+                      |
      | Room SW   | Room S1   | Room S2   |  SE pocket  rubble   |
      |  rubble   |  rubble   | [RADIATION]                      |
  y=-7+-----------+-----------+-----------+----------------------+
     x=-3        x=3         x=8        x=13                   x=17

The purple RADIATION block stands in the north corridor. The LiDAR sees a
1.15 m gap beside it, so the robot plans to drive past it; as soon as the
camera recognises it, its 1.2 m keep-out zone closes the corridor and the
robot turns back the way it came, then reaches the east side of the
building the long way round (south corridor -> east corridor).

Both worlds are named "contaminated_zone" so the beacon node can spawn beacon
models into whichever one is running.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(os.path.dirname(HERE), 'worlds')

WALL_T = 0.2     # wall thickness (m)
WALL_H = 1.0     # wall height (m)

# ------------------------------------------------------------------ big map
# (x1, y1, x2, y2) centre lines. Doors are simply gaps between segments.
# Every segment is lengthened by half a thickness at both ends (clean
# corners), so door gaps below are written 0.2 m wider than the clear
# opening: a 1.4 m gap = a 1.2 m doorway.
WALLS = [
    # outer shell
    (-3, -7, -3, 7), (17, -7, 17, 7), (-3, 7, 17, 7), (-3, -7, 17, -7),
    # y = 3.6 : hall / north corridor north side. Doors: NW, N1, N2
    (-3, 3.6, -0.7, 3.6), (0.7, 3.6, 4.3, 3.6), (5.7, 3.6, 10.3, 3.6), (11.7, 3.6, 13, 3.6),
    # y = -3.6 : hall / south corridor south side. Doors: SW, S1, S2
    (-3, -3.6, -0.7, -3.6), (0.7, -3.6, 4.3, -3.6), (5.7, -3.6, 9.8, -3.6), (11.2, -3.6, 13, -3.6),
    # room dividers
    (3, 3.6, 3, 7), (8, 3.6, 8, 7),
    (3, -7, 3, -3.6), (8, -7, 8, -3.6),
    # x = 13 : east wing wall, doorway at y in [-0.6, 0.6]
    (13, -7, 13, -0.7), (13, 0.7, 13, 7),
    # island (the loop runs around it): west, north, east, south (doorway x in [5.9, 7.1]).
    # North corridor between y = 1.6 and y = 3.6: 1.8 m clear, wide enough to
    # drive past the purple block - only the camera tells the robot not to.
    (3, -2, 3, 1.6), (3, 1.6, 11.3, 1.6), (11.3, -2, 11.3, 1.6),
    (3, -2, 5.8, -2), (7.2, -2, 11.3, -2),
    # island divider, 1.2 m gap at the top
    (7.5, -2, 7.5, 0.2),
    # east wing zigzag (1.2 m gaps at opposite ends)
    (13, 2.5, 15.6, 2.5), (14.4, -2.5, 17, -2.5),
]

# rubble: (name, x, y, size_x, size_y, yaw)
RUBBLE = [
    ('rubble_hall', -2.0, 2.3, 0.6, 0.6, 0.3),
    ('rubble_n1', 6.5, 5.6, 0.8, 0.5, -0.4),
    ('rubble_sw', -1.6, -5.3, 0.7, 0.5, 0.5),
    ('rubble_s1', 5.8, -5.6, 0.6, 0.6, 0.2),
    ('rubble_se', 15.6, -5.0, 0.8, 0.6, -0.3),
    ('rubble_nw', 1.8, 5.8, 0.5, 0.5, 0.0),
]

# hazard blocks the camera detects: (model name, x, y). Size 0.3 x 0.3 x 0.6.
HAZARD_SIZE = (0.3, 0.3, 0.6)
HAZARDS = [
    ('radiation_source', 8.0, 3.0),      # PURPLE, in the north corridor (1.15 m gap south of it)
    ('radiation_source_2', 11.9, -6.0),  # PURPLE, corner of room S2
    ('fire_hazard', 9.6, -0.4),          # RED, inside the island room
    ('gas_leak', 14.3, 5.6),             # YELLOW (new event type), NE pocket
]

BIG_COMMENT = """     Gazebo Jetty (gz-sim) world for the Writer Robot: a 20 m x 14 m damaged
     building with a hall, a loop of corridors around an island room, six
     rooms, a zigzag east wing, rubble, and four hazard blocks for the camera:
       PURPLE  radiation_source    (north corridor: the robot must turn back)
       PURPLE  radiation_source_2  (room S2)
       RED     fire_hazard         (island room)
       YELLOW  gas_leak            (NE pocket, new event type "gas")
     Robot spawn: (0, 0). World name kept as contaminated_zone so the beacon
     node can spawn beacon models into it."""

# ------------------------------------------------------------ retreat test
# Room A (start) -> corridor east -> corner -> corridor north (1.8 m wide),
# where a PURPLE block stands 3 m after the corner, leaving a 0.95 m gap the
# robot could drive through. The camera only sees the block once the robot
# has turned the corner: the robot must turn back the way it came, then map
# room B (north of room A), and finish.
#
#   y=7               +---+
#                     |   |  corridor N (dead end)
#                     | RB|  <- radiation block (6.8, 3.2)
#   y=5 +-----+       |   |
#       |  B  |       |   |
#   y=2 +door-+       |   |
#       |  A  +-------+   |
#       |start  corridor E|
#  y=-2 +-----+-----------+
#      x=-2  x=2    x=5.6 x=7.4
RT_WALLS = [
    # room A (door east y in [-0.6, 0.6], door north x in [-0.6, 0.6])
    (-2, -2, 2, -2), (-2, -2, -2, 2), (2, -2, 2, -0.7), (2, 0.7, 2, 2),
    (-2, 2, -0.7, 2), (0.7, 2, 2, 2),
    # room B
    (-2, 2, -2, 5), (2, 2, 2, 5), (-2, 5, 2, 5),
    # corridor E (y in [-0.8, 0.8]) and corridor N (x in [5.6, 7.4])
    (2, -0.8, 7.4, -0.8), (2, 0.8, 5.6, 0.8),
    (5.6, 0.8, 5.6, 7.0), (7.4, -0.8, 7.4, 7.0), (5.6, 7.0, 7.4, 7.0),
]
RT_RUBBLE = [('rubble_b', -1.2, 4.2, 0.5, 0.5, 0.4)]
RT_HAZARDS = [('radiation_source', 6.8, 3.2)]

RT_COMMENT = """     RETREAT TEST world for the Writer Robot (small, fast on a VM): room A
     (start), room B north of it, and an L-shaped corridor with a PURPLE
     radiation block 3 m after the corner (a 0.95 m gap beside it). Once the
     camera sees it the robot must turn back the way it came, finish mapping
     and return to the start.
     World name kept as contaminated_zone so beacon spawning works."""

# ------------------------------------------------------------------ SDF
MATERIALS = {
    'radiation': ('0.6 0.0 0.6 1', '0.9 0.0 0.9 1', '0.3 0.0 0.3 1'),
    'fire': ('0.6 0.0 0.0 1', '0.95 0.05 0.05 1', '0.4 0.0 0.0 1'),
    'gas': ('0.7 0.6 0.0 1', '1.0 0.85 0.0 1', '0.45 0.38 0.0 1'),
}


def hazard_kind(name):
    for k in MATERIALS:
        if name.startswith(k):
            return k
    raise ValueError(name)


def box_model(name, x, y, z, yaw, sx, sy, sz, ambient, diffuse, emissive=None):
    em = f'<emissive>{emissive}</emissive>' if emissive else ''
    return f"""    <model name="{name}">
      <static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse>{em}</material></visual>
      </link>
    </model>
"""


HEADER = """    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <!-- 4 ms step: light on the CPU, smooth inside a VM -->
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def build(walls, rubble, hazards, comment):
    parts = []
    for i, (x1, y1, x2, y2) in enumerate(walls):
        length = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        # extend by half a thickness at both ends so corners close cleanly
        parts.append(box_model(f'wall_{i:02d}', (x1 + x2) / 2, (y1 + y2) / 2, WALL_H / 2, yaw,
                               length + WALL_T, WALL_T, WALL_H,
                               '0.7 0.7 0.7 1', '0.8 0.8 0.8 1'))
    for name, x, y, sx, sy, yaw in rubble:
        parts.append(box_model(name, x, y, 0.25, yaw, sx, sy, 0.5,
                               '0.5 0.4 0.3 1', '0.6 0.5 0.4 1'))
    sx, sy, sz = HAZARD_SIZE
    for name, x, y in hazards:
        amb, dif, emi = MATERIALS[hazard_kind(name)]
        parts.append(box_model(name, x, y, sz / 2, 0.0, sx, sy, sz, amb, dif, emi))
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_world.py - edit the layout there, then re-run it.
{comment} -->
<sdf version="1.9">
  <world name="contaminated_zone">

{HEADER}
{''.join(parts)}
  </world>
</sdf>
"""


if __name__ == '__main__':
    for fname, walls, rubble, hazards, comment in (
            ('contaminated_zone.world', WALLS, RUBBLE, HAZARDS, BIG_COMMENT),
            ('retreat_test.world', RT_WALLS, RT_RUBBLE, RT_HAZARDS, RT_COMMENT)):
        out = os.path.join(WORLDS, fname)
        with open(out, 'w') as f:
            f.write(build(walls, rubble, hazards, comment))
        print(f'wrote {out}: {len(walls)} walls, {len(rubble)} rubble, {len(hazards)} hazards')
