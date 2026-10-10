#!/usr/bin/env python3
"""
Offline check of the planned tour - no Gazebo, no ROS, runs in a second.

Walks the planned polyline and reports:
  * coverage of every coverable cell, per room and overall
  * any sample where the chassis would overlap a wall or the solid block

Use it after touching house_layout.py or the planner: it catches geometry
mistakes (corner gaps, paths clipping walls) far faster than a 15-minute
simulation. It assumes PERFECT path following, so it is a necessary condition
for 100%, not a sufficient one - real tracking error is what the live run
tests.

Usage:  python3 scripts/verify_coverage.py [room ...]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tb3_house_full_coverage import house_layout as L          # noqa: E402
from tb3_house_full_coverage.coverage_node import build_tour    # noqa: E402

STEP = 0.01          # sampling along each leg (m)
BODY_SAMPLES = 16    # chassis rim points tested for wall overlap


def main():
    rooms = sys.argv[1:] or None
    pts, marks = build_tour(rooms)

    res = L.RESOLUTION
    ox, oy = L.X_MIN - L.MAP_PAD, L.Y_MIN - L.MAP_PAD
    w = int(round((L.X_MAX - L.X_MIN + 2 * L.MAP_PAD) / res))
    h = int(round((L.Y_MAX - L.Y_MIN + 2 * L.MAP_PAD) / res))

    room = [None] * (w * h)
    totals = {}
    for r in range(h):
        y = oy + (r + 0.5) * res
        for c in range(w):
            x = ox + (c + 0.5) * res
            if L.is_occupied(x, y):
                continue
            n = L.room_at(x, y)
            if n is None:
                continue
            room[r * w + c] = n
            totals[n] = totals.get(n, 0) + 1

    wanted = set(rooms) if rooms else set(totals)
    covered = bytearray(w * h)
    done = {n: 0 for n in totals}
    rad = int(math.ceil(L.COVER_RADIUS / res)) + 1

    def mark(x, y):
        rc, cc = int((y - oy) / res), int((x - ox) / res)
        for r in range(max(0, rc - rad), min(h, rc + rad + 1)):
            cy = oy + (r + 0.5) * res
            for c in range(max(0, cc - rad), min(w, cc + rad + 1)):
                k = r * w + c
                if room[k] is None or covered[k]:
                    continue
                cx = ox + (c + 0.5) * res
                if math.hypot(cx - x, cy - y) <= L.COVER_RADIUS:
                    covered[k] = 1
                    done[room[k]] += 1

    collisions = []
    length = 0.0
    mark(*pts[0])
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        length += d
        n = max(1, int(d / STEP))
        for s in range(1, n + 1):
            t = s / n
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            mark(x, y)
            for a in range(BODY_SAMPLES):
                th = a * 2.0 * math.pi / BODY_SAMPLES
                if L.is_occupied(x + L.ROBOT_RADIUS * math.cos(th),
                                 y + L.ROBOT_RADIUS * math.sin(th)):
                    collisions.append((round(x, 2), round(y, 2)))
                    break

    print('rooms      : %s' % (', '.join(rooms) if rooms else 'whole house'))
    print('waypoints  : %d' % len(pts))
    print('path length: %.1f m  (~%.1f min at %.2f m/s)'
          % (length, length / L.V_MAX / 60.0, L.V_MAX))
    print('collisions : %d' % len(collisions))
    for c in collisions[:5]:
        print('    near %s' % (c,))
    print()

    got = tot = 0
    for n in L.TOUR:
        if n not in wanted or not totals.get(n):
            continue
        got += done[n]
        tot += totals[n]
        print('%s %7.2f%%  (%d/%d)'
              % (L.ROOMS[n]['label'], 100.0 * done[n] / totals[n],
                 done[n], totals[n]))
    pct = 100.0 * got / tot if tot else 0.0
    print('REQUESTED  %7.2f%%  (%d/%d)  uncovered=%d' % (pct, got, tot, tot - got))

    ok = not collisions and got == tot
    print('\n%s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
