#!/usr/bin/env python3
"""
Generates worlds/house.world (Gazebo SDF) AND maps/house.pgm + house.yaml
(for map_server) from the shared layout in tb3_house_full_coverage.house_layout,
so the simulation and the localization map can never disagree.

Run from the package root:  python3 scripts/generate_house_world.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tb3_house_full_coverage import house_layout as L  # noqa: E402


def world_sdf() -> str:
    models = []
    for i, (x0, y0, x1, y1) in enumerate(L.WALLS):
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        length = math.hypot(x1 - x0, y1 - y0) + L.WALL_THICK
        yaw = math.atan2(y1 - y0, x1 - x0)
        models.append(f"""
    <model name="wall_{i}">
      <static>true</static>
      <pose>{cx} {cy} {L.WALL_H/2} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length} {L.WALL_THICK} {L.WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length} {L.WALL_THICK} {L.WALL_H}</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.8 1</ambient>
            <diffuse>0.85 0.85 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

    for i, (x0, y0, x1, y1) in enumerate(L.BLOCKS):
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        sx, sy = x1 - x0, y1 - y0
        models.append(f"""
    <model name="block_{i}">
      <static>true</static>
      <pose>{cx} {cy} {L.WALL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {L.WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {L.WALL_H}</size></box></geometry>
          <material>
            <ambient>0.6 0.55 0.5 1</ambient>
            <diffuse>0.6 0.55 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="house_full">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
{''.join(models)}
  </world>
</sdf>
"""


def write_map(pgm_path: str, yaml_path: str) -> None:
    ox, oy = L.X_MIN - L.MAP_PAD, L.Y_MIN - L.MAP_PAD
    w = int(round((L.X_MAX - L.X_MIN + 2 * L.MAP_PAD) / L.RESOLUTION))
    h = int(round((L.Y_MAX - L.Y_MIN + 2 * L.MAP_PAD) / L.RESOLUTION))

    # 254 = free, 0 = occupied (standard map_server PGM convention)
    grid = [[254] * w for _ in range(h)]
    for r in range(h):
        y = oy + (r + 0.5) * L.RESOLUTION
        for c in range(w):
            x = ox + (c + 0.5) * L.RESOLUTION
            if L.is_occupied(x, y):
                grid[r][c] = 0

    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        # PGM row 0 is the TOP of the image; map row 0 is the BOTTOM.
        for r in range(h - 1, -1, -1):
            f.write(bytes(grid[r]))

    with open(yaml_path, 'w') as f:
        f.write(f"""image: house.pgm
resolution: {L.RESOLUTION}
origin: [{ox}, {oy}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")


def main() -> None:
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(os.path.join(pkg, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(pkg, 'maps'), exist_ok=True)

    with open(os.path.join(pkg, 'worlds', 'house.world'), 'w') as f:
        f.write(world_sdf())
    write_map(os.path.join(pkg, 'maps', 'house.pgm'),
              os.path.join(pkg, 'maps', 'house.yaml'))
    print('Wrote worlds/house.world, maps/house.pgm, maps/house.yaml')


if __name__ == '__main__':
    main()
