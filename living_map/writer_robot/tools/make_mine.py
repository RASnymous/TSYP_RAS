#!/usr/bin/env python3
"""
make_mine.py - generates the GAFSA MINE scenario from one layout:

    worlds/gafsa_mine.world                     Gazebo world (rock, rubble, hazard props)
    worlds/gafsa_mine.json                      site file: what the mine sensors measure
    config/ona_gazebo_mine.json                 the ONA's antennas in the mine (robots side)
    ../ona/ona_config_gazebo_mine.json          the same file for the ONA
    ../command_post/public/sites/gafsa_mine.geojson   the mine plan drawn on the dashboard

    python3 tools/make_mine.py

THE SITE. An old room-and-pillar panel ("chambres et piliers") in the Chouabine
phosphate formation, reached by an access drift from the portal, of the kind the
Compagnie des Phosphates de Gafsa worked underground at Metlaoui, Redeyef,
Moulares and M'dhilla until it moved to open pits (about 70 old underground
sites were closed before 2007 and never rehabilitated). The layout is a
SIMULATED panel, not a real CPG plan: galleries 2.8 m wide (scaled to the
robot), 4 m pillars, anchored at the Metlaoui underground mine of the USGS
mineral database (34.3159 N, 8.4184 E).

  y=8.2 +----------------------------------------------------------+
        | north gallery    [ROOF: cracked, slabs]      gem? timber  |
   +--+ |  ___     +----+         +----+         +----+   [VICTIM] |
   |RN| |gw2   |   | P1 |         | P2 |         | P3 |  sits by   |
   |DN| | col  |   +----+         +----+   rich seam  |  the wall  |
   +  + +--     ------------  middle row (main haulage)  ----------+
 portal  drift  ====================================================
  gw1 START  poor seam  +----+  medium seam +----+         +----+   |
   +--------+  |        | P4 |              | P5 |  gw3    | P6 |   |
                |gold    +----+              +----+   col 23 +----+  |
        | south gallery  rubble     [FIRE: burning loader]     [GAS] |
 y=-8.2 +----------------------------------------------------------+
       x=8     9.4           16.2          23.0          29.8   31.2

  RADON  the dead-end gallery north of the drift (no ventilation): radon builds up
         in stagnant air, as in the Abu Tartor underground phosphate mine (Egypt),
         where every measurement exceeded 1000 Bq/m3.
  GAS    blasting fumes and bad air in old workings at the far corner: CO, NO2 and
         an oxygen deficit.
  FIRE   a burning diesel loader in the south gallery (thermal camera).
  ROOF   cracked roof with fallen slabs in the north gallery (roof scanner).
  VICTIM a trapped miner sitting against the east wall (thermal camera).
  PHOSPHATE seams on pillar faces: rich (29 % P2O5), medium (24.5 %), poor (18 %),
         the basin's usual classes (poor 15-22, medium 22-27, rich > 27 %).
  GOLD, GEMSTONE  two DEMONSTRATION targets for the detector: no gold or gemstone
         is documented in the Gafsa phosphate basin.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.dirname(PKG)
sys.path.insert(0, HERE)
from make_world import HEADER  # noqa: E402

ANCHOR = {'lat': 34.3159, 'lon': 8.4184, 'alt': 300.0, 'map_yaw_deg': 0.0}
SITE_NAME = 'Gafsa phosphate basin: simulated room-and-pillar panel (Metlaoui area)'
ROCK_H = 1.6
ROCK = ('0.50 0.45 0.37 1', '0.62 0.56 0.46 1')        # beige-grey phosphate / marl rock

# rock boxes: (name, x0, x1, y0, y1)
ROCK_BOXES = [
    ('rock_portal_gate', -2.1, -1.5, -2.0, 2.0),
    ('rock_drift_s', -2.1, 8.0, -2.0, -1.4),
    ('rock_drift_n_w', -2.1, 3.0, 1.4, 2.0),
    ('rock_drift_n_e', 5.8, 8.0, 1.4, 2.0),
    ('rock_deadend_w', 2.4, 3.0, 1.4, 8.8),
    ('rock_deadend_e', 5.8, 7.4, 2.0, 8.8),
    ('rock_deadend_end', 2.4, 7.4, 8.2, 8.8),
    ('rock_panel_w_n', 7.4, 8.0, 1.4, 8.8),
    ('rock_panel_w_s', 7.4, 8.0, -8.8, -1.4),
    ('rock_panel_n', 7.4, 31.8, 8.2, 8.8),
    ('rock_panel_s', 7.4, 31.8, -8.8, -8.2),
    ('rock_panel_e', 31.2, 31.8, -8.8, 8.8),
    ('pillar_1', 10.8, 14.8, 1.4, 5.4), ('pillar_2', 17.6, 21.6, 1.4, 5.4), ('pillar_3', 24.4, 28.4, 1.4, 5.4),
    ('pillar_4', 10.8, 14.8, -5.4, -1.4), ('pillar_5', 17.6, 21.6, -5.4, -1.4), ('pillar_6', 24.4, 28.4, -5.4, -1.4),
]
# loose rock and old timber the LiDAR sees: (name, x, y, sx, sy, yaw)
RUBBLE = [
    ('rubble_drift', 1.6, -1.0, 0.5, 0.4, 0.3),
    ('rubble_south', 12.8, -7.6, 0.7, 0.5, 0.2),
    ('rubble_col23', 23.9, -3.4, 0.5, 0.5, 0.4),
    ('timber_north', 26.4, 7.75, 0.9, 0.25, 0.08),
]

ITEMS = [
    {'id': 'radon_deadend', 'type': 'radon', 'x': 4.4, 'y': 7.0, 'peak_bqm3': 3200.0, 'spread_m': 2.5,
     'label': 'radon in the unventilated dead-end gallery'},
    {'id': 'bad_air_east', 'type': 'gas', 'x': 29.8, 'y': -6.8, 'spread_m': 1.8,
     'gases': {'co_ppm': 450.0, 'no2_ppm': 14.0, 'h2s_ppm': 0.0, 'ch4_lel': 0.0, 'o2_deficit_pct': 3.4},
     'label': 'blasting fumes and bad air in the old workings'},
    {'id': 'fire_loader', 'type': 'fire', 'x': 19.6, 'y': -7.55, 'temp_c': 420.0,
     'label': 'burning diesel loader'},
    {'id': 'roof_north', 'type': 'roof', 'x': 19.6, 'y': 6.8, 'sag_mm': 45.0,
     'label': 'cracked roof, slabs already fallen'},
    {'id': 'miner_1', 'type': 'victim', 'x': 30.75, 'y': 3.6, 'body_c': 34.0,
     'label': 'trapped miner sitting against the wall'},
    {'id': 'seam_rich', 'type': 'phosphate', 'x0': 24.7, 'y0': 1.4, 'x1': 28.1, 'y1': 1.4,
     'grade_p2o5': 29.0, 'layer': 'C II', 'label': 'rich phosphate layer'},
    {'id': 'seam_medium', 'type': 'phosphate', 'x0': 11.1, 'y0': -1.4, 'x1': 14.5, 'y1': -1.4,
     'grade_p2o5': 24.5, 'layer': 'C V-VI', 'label': 'medium phosphate layer'},
    {'id': 'seam_poor', 'type': 'phosphate', 'x0': 0.4, 'y0': 1.4, 'x1': 2.9, 'y1': 1.4,
     'grade_p2o5': 18.0, 'layer': 'C VII', 'label': 'poor phosphate layer'},
    {'id': 'gold_demo', 'type': 'gold', 'x': 9.2, 'y': -6.3, 'demo': True,
     'label': 'gold: demonstration target (none documented in the Gafsa basin)'},
    {'id': 'gem_demo', 'type': 'gemstone', 'x': 15.6, 'y': 7.5, 'demo': True,
     'label': 'gemstone: demonstration target (none documented in the Gafsa basin)'},
]

# VICTIM SEARCH: the panel cut into the areas a mine rescue team would search one by one.
# The thermal camera "paints" what it has looked at; an area counts as searched at 85 %,
# and the Writer then drops a SEARCHED beacon for it (the digital version of the "X"
# search marking teams spray at a door): "searched, N victims". (id, name, [x0, x1, y0, y1], ...)
ZONES = [
    ('Z1', 'access drift', [(-1.5, 8.0, -1.4, 1.4)]),
    ('Z2', 'dead-end gallery', [(3.0, 5.8, 1.4, 8.2)]),
    ('Z3', 'north gallery', [(8.0, 31.2, 5.4, 8.2)]),
    ('Z4', 'main haulage', [(8.0, 31.2, -1.4, 1.4)]),
    ('Z5', 'south gallery', [(8.0, 31.2, -8.2, -5.4)]),
    ('Z6', 'cross-cuts, column 9', [(8.0, 10.8, 1.4, 5.4), (8.0, 10.8, -5.4, -1.4)]),
    ('Z7', 'cross-cuts, column 16', [(14.8, 17.6, 1.4, 5.4), (14.8, 17.6, -5.4, -1.4)]),
    ('Z8', 'cross-cuts, column 23', [(21.6, 24.4, 1.4, 5.4), (21.6, 24.4, -5.4, -1.4)]),
    ('Z9', 'east gallery, column 30', [(28.4, 31.2, 1.4, 5.4), (28.4, 31.2, -5.4, -1.4)]),
]

# the ONA's antennas in the mine (map frame, z = height): at the portal, and on the
# mine's cable backbone at two junctions (powered and connected from the surface)
GATEWAYS = {
    'gw1': {'local': [-1.0, 0.0, 1.8], 'note': 'portal: sees down the drift and the main haulage row'},
    'gw2': {'local': [9.4, 6.8, 1.8], 'note': 'north-west junction, on the cable backbone'},
    'gw3': {'local': [23.0, -6.8, 1.8], 'note': 'south junction of column 23, on the cable backbone'},
}


# ------------------------------------------------------------------ SDF
def box_sdf(name, cx, cy, cz, yaw, sx, sy, sz, amb, dif, emi=None, collide=True, transparency=0.0):
    em = f'<emissive>{emi}</emissive>' if emi else ''
    tr = f'<transparency>{transparency}</transparency>' if transparency else ''
    col = (f'<collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
           f'</collision>\n        ') if collide else ''
    return f"""    <model name="{name}">
      <static>true</static><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        {col}<visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse>{em}</material>{tr}</visual>
      </link>
    </model>
"""


def disc_sdf(name, x, y, r, amb, dif):
    return f"""    <model name="{name}">
      <static>true</static><pose>{x:.3f} {y:.3f} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><cylinder><radius>{r:.2f}</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse></material><transparency>0.45</transparency></visual>
      </link>
    </model>
"""


def build_world():
    parts = []
    for name, x0, x1, y0, y1 in ROCK_BOXES:
        parts.append(box_sdf(name, (x0 + x1) / 2, (y0 + y1) / 2, ROCK_H / 2, 0.0, x1 - x0, y1 - y0, ROCK_H, *ROCK))
    for name, x, y, sx, sy, yaw in RUBBLE:
        h = 0.35 if name.startswith('timber') else 0.5
        parts.append(box_sdf(name, x, y, h / 2, yaw, sx, sy, h, '0.45 0.38 0.30 1', '0.55 0.47 0.38 1'))
    # props for the eye (the mine sensors read gafsa_mine.json, not these). "deco_" models
    # are visual only: no collision, ignored by the radio model and by the 2D simulators.
    for it in ITEMS:
        t, i = it['type'], it['id']
        if t == 'radon':
            parts.append(disc_sdf(f'deco_{i}', it['x'], it['y'], 1.2, '0.5 0.1 0.6 1', '0.6 0.2 0.8 1'))
        elif t == 'gas':
            parts.append(disc_sdf(f'deco_{i}', it['x'], it['y'], 1.2, '0.6 0.7 0.1 1', '0.75 0.85 0.15 1'))
        elif t == 'fire':
            # the burning loader is a solid machine: the LiDAR sees it
            parts.append(box_sdf(f'mine_loader_{i}', it['x'], it['y'], 0.3, 0.0, 1.0, 0.6, 0.6,
                                 '0.35 0.25 0.05 1', '0.45 0.32 0.08 1'))
            parts.append(box_sdf(f'deco_{i}_flames', it['x'], it['y'], 0.75, 0.0, 0.7, 0.4, 0.3,
                                 '0.9 0.3 0.0 1', '1.0 0.4 0.0 1', '1.0 0.35 0.0 1', collide=False))
        elif t == 'roof':
            for k, (dx, dy, yaw) in enumerate(((-0.5, 0.3, 0.3), (0.4, -0.2, -0.6), (0.1, 0.6, 1.1))):
                parts.append(box_sdf(f'deco_{i}_slab{k}', it['x'] + dx, it['y'] + dy, 0.04, yaw, 0.6, 0.4, 0.08,
                                     '0.40 0.36 0.30 1', '0.5 0.45 0.38 1', collide=False))
            parts.append(box_sdf(f'deco_{i}_warning', it['x'], it['y'], 1.5, 0.0, 2.4, 2.4, 0.03,
                                 '0.6 0.3 0.0 1', '0.8 0.4 0.0 1', collide=False, transparency=0.7))
        elif t == 'victim':
            parts.append(box_sdf(f'victim_{i}', it['x'], it['y'], 0.45, 0.0, 0.5, 0.6, 0.9,
                                 '0.9 0.45 0.0 1', '1.0 0.5 0.0 1'))
            parts.append(box_sdf(f'deco_{i}_helmet', it['x'], it['y'], 0.97, 0.0, 0.25, 0.25, 0.14,
                                 '0.9 0.8 0.0 1', '1.0 0.9 0.0 1', collide=False))
        elif t == 'phosphate':
            x0, y0, x1, y1 = it['x0'], it['y0'], it['x1'], it['y1']
            ln = math.hypot(x1 - x0, y1 - y0)
            yaw = math.atan2(y1 - y0, x1 - x0)
            # a dark band on the rock face, on the gallery side (these faces look towards y = 0)
            off = -0.015 if y0 > 0 else 0.015
            parts.append(box_sdf(f'deco_{i}', (x0 + x1) / 2, (y0 + y1) / 2 + off, 0.6, yaw, ln, 0.03, 0.45,
                                 '0.25 0.25 0.22 1', '0.32 0.31 0.27 1', collide=False))
        elif t == 'gold':
            parts.append(box_sdf(f'deco_{i}', it['x'], it['y'], 0.06, 0.4, 0.15, 0.1, 0.12,
                                 '0.8 0.6 0.1 1', '1.0 0.8 0.2 1', '0.5 0.4 0.05 1', collide=False))
        elif t == 'gemstone':
            parts.append(box_sdf(f'deco_{i}', it['x'], it['y'], 0.05, 0.7, 0.1, 0.1, 0.1,
                                 '0.0 0.6 0.5 1', '0.1 0.9 0.7 1', '0.0 0.4 0.3 1', collide=False))
    comment = ('     GAFSA MINE scenario (generated by tools/make_mine.py): an old room-and-pillar panel\n'
               '     reached by an access drift. Hazards and resources are measured by the mine\n'
               '     sensors from worlds/gafsa_mine.json; the props here are for the eye.\n'
               '     Robot spawn: (0, 0) at the portal. World name kept as contaminated_zone so the\n'
               '     beacon node can spawn beacon models into it.')
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_mine.py - edit the layout there, then re-run it.
{comment} -->
<sdf version="1.9">
  <world name="contaminated_zone">

{HEADER}
{''.join(parts)}
  </world>
</sdf>
"""


# ------------------------------------------------------------------ site file, plan, ONA config
def to_latlon(x, y):
    yaw = math.radians(ANCHOR['map_yaw_deg'])
    e = x * math.cos(yaw) - y * math.sin(yaw)
    n = x * math.sin(yaw) + y * math.cos(yaw)
    lat = ANCHOR['lat'] + n / 111320.0
    lon = ANCHOR['lon'] + e / (111320.0 * math.cos(math.radians(ANCHOR['lat'])))
    return [round(lon, 7), round(lat, 7)]


def site_json():
    return {
        'format': 'living-map-site/1',
        'name': SITE_NAME,
        'world': 'gafsa_mine.world',
        'anchor': ANCHOR,
        'note': ('Simulated layout. Gas and radon spread along the galleries (walking distance), never '
                 'through rock. Gold and gemstone are demonstration targets only.'),
        'thresholds': {'co_ppm': 100.0, 'no2_ppm': 5.0, 'h2s_ppm': 10.0, 'ch4_lel': 20.0, 'o2_min_pct': 19.5,
                       'radon_bqm3': 1000.0, 'radon_info_bqm3': 300.0},
        'background': {'radon_bqm3': 60.0, 'gamma_usvh': 0.08, 'gamma_ore_usvh': 0.22, 'temp_c': 24.0},
        'items': ITEMS,
        'search': {'camera': 'thermal', 'hfov_deg': 57.0, 'range_m': 6.0, 'done_fraction': 0.85},
        'zones': [{'id': z, 'name': n, 'rects': [list(r) for r in rects]} for z, n, rects in ZONES],
    }


def plan_geojson():
    feats = []
    for name, x0, x1, y0, y1 in ROCK_BOXES:
        ring = [to_latlon(x0, y0), to_latlon(x1, y0), to_latlon(x1, y1), to_latlon(x0, y1), to_latlon(x0, y0)]
        feats.append({'type': 'Feature', 'properties': {'kind': 'pillar' if name.startswith('pillar') else 'rock',
                                                         'name': name},
                      'geometry': {'type': 'Polygon', 'coordinates': [ring]}})
    for it in ITEMS:
        if it['type'] == 'phosphate':
            feats.append({'type': 'Feature', 'properties': {'kind': 'seam', 'name': it['label'],
                                                             'grade_p2o5': it['grade_p2o5'], 'layer': it['layer']},
                          'geometry': {'type': 'LineString',
                                       'coordinates': [to_latlon(it['x0'], it['y0']), to_latlon(it['x1'], it['y1'])]}})
    for z, n, rects in ZONES:
        polys = [[[to_latlon(x0, y0), to_latlon(x1, y0), to_latlon(x1, y1), to_latlon(x0, y1), to_latlon(x0, y0)]]
                 for x0, x1, y0, y1 in rects]
        feats.append({'type': 'Feature', 'properties': {'kind': 'zone', 'id': z, 'name': n},
                      'geometry': {'type': 'MultiPolygon', 'coordinates': polys}})
    for g, v in GATEWAYS.items():
        feats.append({'type': 'Feature', 'properties': {'kind': 'gateway', 'name': g, 'note': v['note']},
                      'geometry': {'type': 'Point', 'coordinates': to_latlon(v['local'][0], v['local'][1])}})
    feats.append({'type': 'Feature', 'properties': {'kind': 'portal', 'name': 'Portal (entrance, EXIT)'},
                  'geometry': {'type': 'Point', 'coordinates': to_latlon(0.0, 0.0)}})
    return {'type': 'FeatureCollection', 'name': SITE_NAME,
            'properties': {'site': 'gafsa_mine', 'name': SITE_NAME, 'underground': True,
                           'note': 'Simulated room-and-pillar layout, not a CPG plan.'},
            'features': feats}


def ona_config():
    return {
        'name': 'ONA-Gafsa-mine',
        'site': {'id': 'gafsa_mine', 'name': SITE_NAME, 'plan': 'sites/gafsa_mine.geojson', 'underground': True},
        'key': 'demo', 'net_id': 42, 'quorum': 2,
        'command_post': 'http://10.0.2.2:3000',
        'anchor': ANCHOR,
        'gateways': GATEWAYS,
        'radio': {'medium': 'rock', 'rock_db_per_m': 8.0, 'tof_needs_los': True},
        'tracker': {'robot_height_m': 0.3, 'ping_window_s': 1.0, 'rssi_ranging': False},
        'unconfirmed_hold_s': 4.0, 'strict': False,
    }


def main():
    out = []
    w = os.path.join(PKG, 'worlds', 'gafsa_mine.world')
    with open(w, 'w') as f:
        f.write(build_world())
    out.append(w)
    s = os.path.join(PKG, 'worlds', 'gafsa_mine.json')
    with open(s, 'w') as f:
        json.dump(site_json(), f, indent=1)
    out.append(s)
    for p in (os.path.join(PKG, 'config', 'ona_gazebo_mine.json'),
              os.path.join(ROOT, 'ona', 'ona_config_gazebo_mine.json')):
        if os.path.isdir(os.path.dirname(p)):
            with open(p, 'w') as f:
                json.dump(ona_config(), f, indent=1)
            out.append(p)
    g = os.path.join(ROOT, 'command_post', 'public', 'sites')
    if os.path.isdir(os.path.dirname(g)):
        os.makedirs(g, exist_ok=True)
        with open(os.path.join(g, 'gafsa_mine.geojson'), 'w') as f:
            json.dump(plan_geojson(), f)
        out.append(os.path.join(g, 'gafsa_mine.geojson'))
    for p in out:
        print('wrote', p)


if __name__ == '__main__':
    main()
