#!/usr/bin/env python3
"""
Offline check that EVERY room subset is fully covered, not just the full tour.

`verify_coverage.py` checks one request at a time. This walks all of them -
each room alone, every pair, and the whole house - because the promise
`tb3_voice_coverage` makes is per-request: whatever rooms you name get swept
completely. A planner change can easily keep the full-house tour at 100% and
still break a subset, since a subset transits different floor and so reaches
each room with a different set of cells already swept.

The denominator is the REQUESTED rooms only. Floor swept in passing on the way
there is a bonus, not the thing being verified.

Usage:  python3 scripts/verify_subsets.py [--triples]
"""

import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tb3_house_full_coverage import house_layout as L            # noqa: E402
from tb3_house_full_coverage import coverage_node as C           # noqa: E402

STEP = 0.02          # sampling along each leg (m)
BODY_SAMPLES = 16    # chassis rim points tested for wall overlap


def check(rooms):
    """-> (path length, cells requested, cells missed, wall-contact samples)."""
    pts, _ = C.build_tour(list(rooms))
    sim = C._CoverageSim()

    need = set()
    for name in rooms:
        for rect in L.ROOMS[name]['rects']:
            need |= sim.rect_cells(rect)

    sim.commit(sim._cells(*pts[0]))
    dist, hits = 0.0, 0
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        dist += d
        sim.commit(sim.preview(a, b))
        n = max(1, int(d / STEP))
        for s in range(n + 1):
            t = s / n
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            for k in range(BODY_SAMPLES):
                th = k * math.pi / BODY_SAMPLES * 2
                if L.is_occupied(x + L.ROBOT_RADIUS * math.cos(th),
                                 y + L.ROBOT_RADIUS * math.sin(th)):
                    hits += 1
                    break
    return dist, len(need), sum(1 for k in need if not sim.covered[k]), hits


def main():
    names = L.TOUR
    combos = [(r,) for r in names]
    combos += list(itertools.combinations(names, 2))
    if '--triples' in sys.argv:
        combos += list(itertools.combinations(names, 3))
    combos.append(tuple(names))

    print('%-40s %9s %8s %6s %6s' % ('request', 'path', 'cells', 'miss', 'hits'))
    fails = 0
    for rooms in combos:
        dist, total, miss, hits = check(rooms)
        ok = miss == 0 and hits == 0
        fails += not ok
        print('%-40s %8.1fm %8d %6d %6d%s'
              % ('+'.join(rooms)[:40], dist, total, miss, hits,
                 '' if ok else '   <-- FAIL'))

    print('\n%d combinations, %d failures' % (len(combos), fails))
    print('PASS' if not fails else 'FAIL')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
