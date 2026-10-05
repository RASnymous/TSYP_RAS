#!/usr/bin/env python3
"""
mission_world.py - builds the Gazebo world for the Executor: the same world the
Writer explored, plus every beacon the Writer dropped, lying where it fell.

    ros2 run writer_robot mission_world --mission ~/writer_robot_ws/missions/latest.json \
        --out /tmp/executor_world.world
    (or: python3 -m writer_robot.mission_world ...)

The beacons are static, visual-only models (no collision, below the LiDAR),
so they never get in any robot's way. All have a green body; the LEDs are
blue, and white on the EXIT beacon. None of these colours can be mistaken
for a hazard by the camera.
The base world is taken from the mission file ("world"), looked up in the
package's worlds/ folder (or give --world).
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))


def worlds_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('writer_robot'), 'worlds')
        if os.path.isdir(d):
            return d
    except Exception:  # noqa: BLE001 - not in a ROS environment
        pass
    return os.path.join(os.path.dirname(HERE), 'worlds')


def beacon_sdf(name, x, y, exit_):
    led = ('1.0 1.0 1.0 1', '0.9 0.9 0.9 1') if exit_ else ('0.1 0.3 1.0 1', '0.0 0.2 1.0 1')
    return f"""    <model name="{name}">
      <static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="body"><pose>0 0 0.04 0 0 0</pose><geometry><box><size>0.14 0.10 0.08</size></box></geometry>
          <material><ambient>0.0 0.9 0.2 1</ambient><diffuse>0.1 1.0 0.3 1</diffuse><emissive>0.0 0.8 0.2 1</emissive></material></visual>
        <visual name="antenna"><pose>0.05 0 0.12 0 0 0</pose><geometry><cylinder><radius>0.004</radius><length>0.08</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
        <visual name="led"><pose>-0.03 0 0.095 0 0 0</pose><geometry><sphere><radius>0.02</radius></sphere></geometry>
          <material><ambient>{led[0]}</ambient><diffuse>{led[0]}</diffuse><emissive>{led[1]}</emissive></material></visual>
      </link>
    </model>
"""


def build(mission, base_world):
    tree = ET.parse(base_world)
    world = tree.getroot().find('world')
    n = 0
    for e in mission.get('beacons', []):
        x, y = e['drop']
        kind = e.get('record', {}).get('kind', '')
        world.append(ET.fromstring(beacon_sdf(f'beacon_{e["id"]}', float(x), float(y), kind == 'EXIT')))
        n += 1
    return tree, n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mission', default=os.path.expanduser('~/writer_robot_ws/missions/latest.json'))
    ap.add_argument('--world', help='base world file (default: the one named in the mission)')
    ap.add_argument('--out', default='/tmp/executor_world.world')
    a = ap.parse_args(argv)
    with open(os.path.expanduser(a.mission)) as f:
        mission = json.load(f)
    base = a.world or os.path.join(worlds_dir(), mission.get('world') or 'contaminated_zone.world')
    if not os.path.exists(base):
        print(f'base world not found: {base}', file=sys.stderr)
        return 1
    tree, n = build(mission, base)
    tree.write(a.out, xml_declaration=True, encoding='unicode')
    print(f'{a.out}: {os.path.basename(base)} + {n} beacons')
    return 0


if __name__ == '__main__':
    sys.exit(main())
