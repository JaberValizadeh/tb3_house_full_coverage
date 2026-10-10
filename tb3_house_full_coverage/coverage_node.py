#!/usr/bin/env python3
"""
coverage_node - one robot sweeping the whole house, room after room.

Plans a single ordered list of waypoints for the entire tour, then follows
it. For each room it lays boustrophedon (back-and-forth) lanes across every
rectangle of that room, and moves between rectangles, rooms and the corridor
through hand-checked door waypoints - so every straight segment stays inside
free space and never clips a wall.

The lane geometry is what makes 100% coverage reachable:
  * lanes are inset by MARGIN, the closest a robot centre may legally get to
    a wall, and the swept band is COVER_RADIUS > MARGIN, so the band still
    reaches past the wall face;
  * lane spacing LANE_W <= 2 * COVER_RADIUS, so neighbouring lanes overlap.

Poses come from Gazebo ground truth (/gazebo/model_states), like the other
tb3 coverage packages - AMCL in localization.launch is shown on the map but
is not the controller's input.
"""

import math

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

from tb3_house_full_coverage import house_layout as L

# Waypoint follower gains
#
# The robot must stay ON the planned line, not merely arrive near each
# waypoint: coverage is computed from the swept band, and cells at the band's
# outer edge (doorway thresholds against a wall line) are lost by a few cm of
# drift. Accepting a waypoint within POS_TOL lets the next leg START that far
# off, so without cross-track correction the driven line is offset from the
# planned one by roughly POS_TOL all the way along.
POS_TOL = 0.03        # waypoint reached (m)
ALIGN_TOL = 0.12      # turn in place until heading error is under this (rad)
K_ANG = 2.2
K_LIN = 1.2
K_CROSS = 2.5         # pull back onto the planned line (rad per m of offset)
MAX_CROSS_CORR = 0.6  # cap that correction (rad)
RATE_HZ = 20.0


def _seg_clear(a, b, step=0.02):
    """True if the chassis clears every wall along the straight segment a->b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    n = max(1, int(d / step))
    for s in range(n + 1):
        t = s / n
        x, y = a[0] + dx * t, a[1] + dy * t
        for k in range(16):
            th = k * math.pi / 8
            if L.is_occupied(x + L.ROBOT_RADIUS * math.cos(th),
                             y + L.ROBOT_RADIUS * math.sin(th)):
                return False
    return True


def manhattanize(pts):
    """Replace diagonal legs with axis-aligned ones via a safe corner.

    Lane and perimeter legs are already axis-aligned; this only affects the
    transitions between lanes, bridges and doors. A diagonal shortcut there
    can cut the corner of a wall it passes - inserting the corner keeps every
    leg parallel to the walls it runs alongside.

    Returns (point, source_index) pairs so room boundaries can be re-mapped
    onto the expanded list. Mapping by point VALUE does not work: waypoints
    repeat all over the tour, so a value-keyed lookup collapses them.
    """
    out = [(pts[0], 0)]
    for i, p in enumerate(pts[1:], 1):
        c = out[-1][0]
        if abs(p[0] - c[0]) > 1e-9 and abs(p[1] - c[1]) > 1e-9:
            for cand in ((p[0], c[1]), (c[0], p[1])):
                if _seg_clear(c, cand) and _seg_clear(cand, p):
                    out.append((cand, i))
                    break
        out.append((p, i))
    return out


def insets(rect):
    """Smallest safe inset for each edge: (left, bottom, right, top).

    A uniform MARGIN assumes every rectangle edge is a wall CENTRELINE, so it
    reserves half a wall thickness that isn't always there. The study's arms
    are bounded by the solid block's FACE, and seam edges have no wall at all -
    insetting 0.21 from those leaves an unreachable pocket. Probing the real
    clearance lets the sweep hug whatever each edge actually is.
    """
    x0, y0, x1, y1 = rect
    span_y = (y0 + L.MARGIN, y1 - L.MARGIN)
    span_x = (x0 + L.MARGIN, x1 - L.MARGIN)
    if span_y[1] <= span_y[0]:
        span_y = ((y0 + y1) / 2.0,) * 2
    if span_x[1] <= span_x[0]:
        span_x = ((x0 + x1) / 2.0,) * 2

    lo = L.ROBOT_RADIUS + 0.02
    cands = [lo + 0.005 * i for i in range(int((L.MARGIN - lo) / 0.005) + 1)]
    cands.append(L.MARGIN)

    out = []
    for edge in range(4):                     # left, bottom, right, top
        chosen = L.MARGIN
        for d in cands:
            if edge == 0:
                a, b = (x0 + d, span_y[0]), (x0 + d, span_y[1])
            elif edge == 2:
                a, b = (x1 - d, span_y[0]), (x1 - d, span_y[1])
            elif edge == 1:
                a, b = (span_x[0], y0 + d), (span_x[1], y0 + d)
            else:
                a, b = (span_x[0], y1 - d), (span_x[1], y1 - d)
            if _seg_clear(a, b):
                chosen = d
                break
        out.append(chosen)
    return out


def sweep_box(rect):
    """The rectangle shrunk by its per-edge safe insets."""
    x0, y0, x1, y1 = rect
    dl, db, dr, dt = insets(rect)
    return x0 + dl, y0 + db, x1 - dr, y1 - dt


def perimeter_waypoints(rect):
    """One lap around the inside of the rectangle, inset by MARGIN.

    Without this, the strip between a lane's end and the wall is only ever
    touched by the discs at the lane ends, and cells in that strip that sit
    between two lanes fall just outside the swept band - the classic
    boustrophedon corner gap. One edge-following lap covers the whole
    boundary strip continuously, which is also how real cleaning robots do
    edges before filling the middle.
    """
    ax0, ay0, ax1, ay1 = sweep_box(rect)
    if ax1 <= ax0 or ay1 <= ay0:
        return []
    return [(ax0, ay0), (ax1, ay0), (ax1, ay1), (ax0, ay1), (ax0, ay0)]


def _inward(pt, rect):
    """Nearest point that is at least MARGIN inside `rect`.

    Bridge waypoints sit exactly on the edge shared by two rectangles. Driving
    from one bridge straight to the next would run ALONG that shared edge -
    which, in the U-shaped study, goes straight through the solid block.
    Stepping inward first keeps every crossing perpendicular to the edge.
    """
    x0, y0, x1, y1 = rect
    return (min(max(pt[0], x0 + L.MARGIN), x1 - L.MARGIN),
            min(max(pt[1], y0 + L.MARGIN), y1 - L.MARGIN))


def lane_waypoints(rect, skip_outer=False, horizontal=None):
    """Boustrophedon lanes covering one rectangle, inset by MARGIN.

    With `skip_outer`, the first and last lanes are dropped: they lie exactly
    on the perimeter lap's straight legs, so driving them again is pure
    duplication.

    `horizontal` forces the lane direction. Left unset it follows the longer
    side - fewer turns, straighter runs - but which direction actually costs
    less depends on where the robot enters, so `_choose_sweep` tries both.
    """
    w, h = rect[2] - rect[0], rect[3] - rect[1]
    bx0, by0, bx1, by1 = sweep_box(rect)
    # Sweep along the longer side: fewer turns, straighter runs.
    if horizontal is None:
        horizontal = w >= h
    if horizontal:
        lo, hi = by0, by1
        a0, a1 = bx0, bx1
    else:
        lo, hi = bx0, bx1
        a0, a1 = by0, by1

    # Degenerate strip: one lane down the middle.
    if hi <= lo:
        lo = hi = (lo + hi) / 2.0
        n = 1
    else:
        n = max(2, int(math.ceil((hi - lo) / L.LANE_W)) + 1)
    if a1 <= a0:
        a0 = a1 = (a0 + a1) / 2.0

    idxs = range(n)
    if skip_outer and n >= 2:
        idxs = range(1, n - 1)

    pts = []
    for i in idxs:
        p = lo if n == 1 else lo + i * (hi - lo) / (n - 1)
        lo_end, hi_end = (a0, a1) if i % 2 == 0 else (a1, a0)
        if horizontal:
            pts.append((lo_end, p))
            pts.append((hi_end, p))
        else:
            pts.append((p, lo_end))
            pts.append((p, hi_end))
    return pts


def room_waypoints(name, start=None, sim=None):
    """Enter the room, sweep every rectangle, come back out to the corridor.

    With a `_CoverageSim`, each rectangle's sweep is *chosen* rather than
    assumed (see `_choose_sweep`) and every leg is committed to `sim`, so the
    choice for a later rectangle knows what the earlier ones already swept.
    Without one the original lap-then-lanes sweep is used unchanged.
    """
    room = L.ROOMS[name]
    access = list(room['access'])
    bridges = list(room['bridges'])
    rects = room['rects']

    pts = list(access)                       # corridor -> inside the room
    cur = access[-1] if access else start
    if sim is not None:
        prev = start
        for p in access:
            if prev is not None:
                _credit(sim, prev, p)
            prev = p

    for i, rect in enumerate(rects):
        if i > 0:                            # cross into the next rectangle
            b = bridges[i - 1]
            for p in (b, _inward(b, rect)):
                if sim is not None and cur is not None:
                    _credit(sim, cur, p)
                pts.append(p)
                cur = p
        if sim is not None and cur is not None:
            seq = _choose_sweep(rect, cur, sim)
        else:
            ring = perimeter_waypoints(rect)
            seq = ring + lane_waypoints(rect, skip_outer=bool(ring))
        pts.extend(seq)
        if seq:
            cur = seq[-1]

    # Retrace to the rectangle we came in through, stepping inside each
    # rectangle rather than sliding along the shared edges.
    tail = []
    for i in range(len(rects) - 1, 0, -1):
        b = bridges[i - 1]
        tail.extend([_inward(b, rects[i]), b, _inward(b, rects[i - 1])])
    tail.extend(reversed(access))            # back out to the corridor
    if sim is not None:
        for p in tail:
            if cur is not None:
                _credit(sim, cur, p)
            cur = p
    pts.extend(tail)
    return pts


class _CoverageSim:
    """Tracks which cells the swept band has already reached.

    Used at plan time so the planner can tell whether a leg would actually
    clean anything. Without it every lane and perimeter lap is driven
    unconditionally, including over ground that transits already covered.

    `covered` seeds the state with floor that is *already* clean - the live
    grid from the coordinator, say. A replan then plans around it instead of
    re-driving it, which is what lets a second request, or a top-up pass after
    an interrupted one, cost only the floor still outstanding.
    """

    def __init__(self, covered=None):
        self.res = L.RESOLUTION
        self.ox = L.X_MIN - L.MAP_PAD
        self.oy = L.Y_MIN - L.MAP_PAD
        self.w = int(round((L.X_MAX - L.X_MIN + 2 * L.MAP_PAD) / self.res))
        self.h = int(round((L.Y_MAX - L.Y_MIN + 2 * L.MAP_PAD) / self.res))
        self.rad = int(math.ceil(L.COVER_RADIUS / self.res)) + 1
        self.coverable = bytearray(self.w * self.h)
        for r in range(self.h):
            y = self.oy + (r + 0.5) * self.res
            for c in range(self.w):
                x = self.ox + (c + 0.5) * self.res
                if not L.is_occupied(x, y) and L.room_at(x, y) is not None:
                    self.coverable[r * self.w + c] = 1
        self.covered = bytearray(self.w * self.h)
        if covered:
            self.seed(covered)

    def seed(self, cells):
        """Mark an iterable of cell indices as already swept."""
        for k in cells:
            if 0 <= k < len(self.covered):
                self.covered[k] = 1

    def cells_of(self, rects):
        """Every coverable cell inside `rects`, covered or not."""
        out = set()
        for rect in rects:
            x0, y0, x1, y1 = rect
            c0 = max(0, int((x0 - self.ox) / self.res))
            c1 = min(self.w - 1, int((x1 - self.ox) / self.res))
            r0 = max(0, int((y0 - self.oy) / self.res))
            r1 = min(self.h - 1, int((y1 - self.oy) / self.res))
            for r in range(r0, r1 + 1):
                y = self.oy + (r + 0.5) * self.res
                if not y0 <= y <= y1:
                    continue
                for c in range(c0, c1 + 1):
                    k = r * self.w + c
                    if not self.coverable[k]:
                        continue
                    x = self.ox + (c + 0.5) * self.res
                    if x0 <= x <= x1:
                        out.add(k)
        return out

    def _cells(self, x, y):
        rc, cc = int((y - self.oy) / self.res), int((x - self.ox) / self.res)
        out = []
        for r in range(max(0, rc - self.rad), min(self.h, rc + self.rad + 1)):
            cy = self.oy + (r + 0.5) * self.res
            for c in range(max(0, cc - self.rad),
                           min(self.w, cc + self.rad + 1)):
                k = r * self.w + c
                if not self.coverable[k] or self.covered[k]:
                    continue
                cx = self.ox + (c + 0.5) * self.res
                if math.hypot(cx - x, cy - y) <= L.COVER_RADIUS:
                    out.append(k)
        return out

    def preview(self, a, b, step=0.02):
        """Cells that driving a->b would newly cover (nothing committed)."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = max(1, int(math.hypot(dx, dy) / step))
        fresh = set()
        for s in range(n + 1):
            t = s / n
            for k in self._cells(a[0] + dx * t, a[1] + dy * t):
                fresh.add(k)
        return fresh

    def commit(self, cells):
        for k in cells:
            self.covered[k] = 1

    def rect_cells(self, rect):
        """Coverable cells inside `rect` that are still unswept."""
        return set(k for k in self.cells_of([rect]) if not self.covered[k])


def _credit(sim, a, b):
    """Commit what driving a->b certainly sweeps, and no more.

    A diagonal leg is only a plan: `manhattanize` turns it into two
    axis-aligned legs via a corner, so the band it really sweeps is not the
    one along a->b. Crediting the diagonal would let the planner believe floor
    is done that the robot never passes, so a diagonal is credited only with
    the discs at its two ends - which it does sweep either way.
    """
    if abs(a[0] - b[0]) < 1e-9 or abs(a[1] - b[1]) < 1e-9:
        sim.commit(sim.preview(a, b))
    else:
        sim.commit(sim._cells(*a))
        sim.commit(sim._cells(*b))


def _rect_candidates(rect):
    """Every sweep worth considering for one rectangle.

    The perimeter lap exists to close boustrophedon corner gaps, but in a
    rectangle narrow enough for the lanes alone to reach both walls it is a
    second full-length run over floor the lanes already cover - which is
    exactly what made the corridor and the study's bottom bar the two most
    repeated strips in the house. So the lap is offered as an option, not
    imposed, and the same goes for lane direction and traversal order.
    """
    ring = perimeter_waypoints(rect)
    cands = []
    if ring:
        cands.append(ring)
    for horizontal in (True, False):
        lanes = lane_waypoints(rect, horizontal=horizontal)
        if lanes:
            cands.append(lanes)
        if ring:
            inner = lane_waypoints(rect, skip_outer=True, horizontal=horizontal)
            cands.append(ring + inner)
            if inner:
                cands.append(ring + list(reversed(inner)))
    # Entering near a sweep's far end costs a whole extra traverse, so let the
    # chooser start from either end.
    return cands + [list(reversed(c)) for c in cands]


def _choose_sweep(rect, entry, sim):
    """Shortest candidate sweep of `rect` that still covers all of it.

    "Covers all of it" is checked against the simulator, not argued from
    geometry: every coverable cell inside `rect` that is not already swept
    must be swept by the candidate. A candidate that misses even one is
    discarded, so choosing the cheapest can never cost coverage. Falls back
    to lap-then-lanes if nothing qualifies.
    """
    need = sim.rect_cells(rect)
    best = None
    for cand in _rect_candidates(rect):
        path = [entry] + cand
        got, dist = set(), 0.0
        for i, (a, b) in enumerate(zip(path, path[1:])):
            if not _seg_clear(a, b):
                dist = None
                break
            dist += math.hypot(b[0] - a[0], b[1] - a[1])
            # The leg INTO the sweep is paid for but never credited: it is the
            # one leg that may be diagonal, and `manhattanize` will later
            # replace any diagonal with two axis-aligned legs sweeping a
            # different band. Crediting it made the chooser pick sweeps that
            # only covered the rectangle along a line the robot never drives -
            # seven living-room cells were lost exactly that way.
            if i:
                got |= sim.preview(a, b, step=0.05)
        if dist is None or (need - got):
            continue
        if best is None or dist < best[0]:
            best = (dist, cand)

    if best is None:
        ring = perimeter_waypoints(rect)
        chosen = ring + lane_waypoints(rect, skip_outer=bool(ring))
    else:
        chosen = best[1]

    cur = entry
    for p in chosen:
        _credit(sim, cur, p)
        cur = p
    return chosen


def door_x(name):
    """Where a room's access chain meets the corridor centreline, else None.

    Every room is entered from the corridor, so the cost of travelling between
    two rooms is essentially the gap between their doors along x. The corridor
    itself has no access chain - it *is* the transit space.
    """
    access = L.ROOMS[name]['access']
    return access[0][0] if access else None


def order_rooms(rooms, start=None):
    """`rooms` in the order that costs the least driving from `start`.

    All doors sit on the corridor centreline, so the transit part of a
    multi-room request is a one-dimensional problem: visit a set of points on
    a line, starting at the robot. The cheapest open path always runs to the
    nearer end of the span first and then straight across, so the whole thing
    is solved exactly rather than approximated - no room is passed twice.

    House order (`house_layout.TOUR`) is the special case of this for a robot
    parked at the west end, which is where it spawns. It stops being the
    cheapest as soon as the robot is somewhere else: after cleaning the
    bathroom, "the kitchen and the bathroom" in house order drives the length
    of the house twice.

    The corridor is always swept last - it is the space every other room is
    reached through, so by then it is largely done already.
    """
    rooms = list(rooms)
    rank = {n: i for i, n in enumerate(L.TOUR)}
    tail = [n for n in rooms if door_x(n) is None]
    doors = [(door_x(n), rank.get(n, 0), n) for n in rooms if door_x(n) is not None]
    if not doors:
        return sorted(tail, key=lambda n: rank.get(n, 0))

    sx = start[0] if start is not None else L.SPAWN[0]
    lo = min(d for d, _, _ in doors)
    hi = max(d for d, _, _ in doors)

    # Rooms sharing a door - the store is reached through the living room -
    # tie-break into house order whichever way the robot is travelling, so
    # the outer room is swept on the way in rather than crossed twice.
    def west(d):        # right to left
        return (-d[0], d[1])

    def east(d):        # left to right
        return (d[0], d[1])

    # Going for the near end first costs (sx-lo) + (hi-lo); the far end first
    # costs (hi-sx) + (hi-lo). Only the first term differs.
    if (sx - lo) <= (hi - sx):
        seq = (sorted([d for d in doors if d[0] <= sx], key=west)
               + sorted([d for d in doors if d[0] > sx], key=east))
    else:
        seq = (sorted([d for d in doors if d[0] >= sx], key=east)
               + sorted([d for d in doors if d[0] < sx], key=west))
    return [n for _, _, n in seq] + sorted(tail, key=lambda n: rank.get(n, 0))


def prune(pts, marks, covered=None):
    """Drop legs that would clean nothing.

    A leg is only kept if it either reveals new floor or is needed to get
    somewhere. Skipping is allowed only when the resulting shortcut is
    collision-free, so pruning can never route the robot through a wall.

    This is what stops the robot re-driving lines it has already swept - most
    visibly the corridor, which every room transit crosses long before the
    corridor's own sweep is planned. Seeded with `covered`, it does the same
    for floor cleaned before this plan was made.
    """
    sim = _CoverageSim(covered)
    sim.commit(sim._cells(*pts[0]))
    out, src = [pts[0]], [0]

    i = 1
    while i < len(pts):
        fresh = sim.preview(out[-1], pts[i])
        last = i == len(pts) - 1

        if not fresh and not last and _seg_clear(out[-1], pts[i + 1]):
            # Dropping pts[i] also re-routes the NEXT leg, so it is only safe
            # if the shortcut still sweeps everything that leg would have.
            # Checking just "this leg cleans nothing" silently loses cells.
            would_lose = sim.preview(pts[i], pts[i + 1])
            shortcut = sim.preview(out[-1], pts[i + 1])
            if would_lose <= shortcut:
                i += 1                 # nothing to clean here - skip it
                continue

        sim.commit(fresh)
        out.append(pts[i])
        src.append(i)
        i += 1

    remapped = []
    for name, a, b in marks:
        ia = next((j for j, s in enumerate(src) if s >= a), len(out) - 1)
        ib = next((j for j, s in enumerate(src) if s >= b), len(out) - 1)
        remapped.append((name, ia, ib))
    return out, remapped


def build_tour(rooms=None, optimize=True, start=None, covered=None):
    """An ordered waypoint list over `rooms`, with room boundaries.

    Defaults to the whole house. Any subset works because every room's access
    chain starts and ends on the corridor centreline, so consecutive rooms are
    joined by a straight run along the corridor.

    `start` is where the robot actually is (default: the spawn pose), and
    `covered` the cell indices already swept. Both only ever *remove* driving:
    the sweep chosen for each rectangle still has to cover every one of its
    cells that `covered` does not already account for, so a shorter plan is
    never a less complete one.
    """
    # One simulator spans the whole tour, so each room's sweep is chosen
    # knowing what the rooms before it - and the transits between them -
    # already swept.
    sim = _CoverageSim(covered) if optimize else None
    pos = tuple(start[:2]) if start is not None else L.SPAWN[:2]
    if sim is not None:
        sim.commit(sim._cells(*pos))

    pts, marks = [], []
    for name in (rooms if rooms is not None else L.TOUR):
        start = len(pts)
        seq = room_waypoints(name, pos, sim)
        if seq:
            pos = seq[-1]
        pts.extend(seq)
        marks.append((name, start, len(pts)))

    # Drop consecutive duplicates (entering/leaving through the same door).
    clean, clean_marks = [], []
    idx_map = {}
    for i, p in enumerate(pts):
        if not clean or math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-6:
            clean.append(p)
        idx_map[i] = len(clean) - 1
    for name, a, b in marks:
        clean_marks.append((name, idx_map[a], idx_map[min(b, len(pts) - 1)]))

    # Keep every leg parallel to the walls, then re-map the room boundaries
    # onto the expanded list via source indices.
    pairs = manhattanize(clean)
    final = [p for p, _ in pairs]
    src = [s for _, s in pairs]

    def first_at(idx):
        for i, s in enumerate(src):
            if s >= idx:
                return i
        return len(final) - 1

    remapped = [(name, first_at(a), first_at(b)) for name, a, b in clean_marks]
    if optimize:
        final, remapped = prune(final, remapped, covered)
    return final, remapped


def steer(pose, leg_start, target):
    """Drive from `leg_start` toward waypoint `target`; -> (Twist, reached).

    Shared with tb3_voice_coverage so the cross-track behaviour that the 100%
    result depends on lives in exactly one place.
    """
    x, y, yaw = pose
    dx, dy = target[0] - x, target[1] - y
    dist = math.hypot(dx, dy)
    if dist < POS_TOL:
        return Twist(), True

    # Aim along the planned leg, corrected by how far off it we are, so the
    # driven line converges onto the planned one instead of running parallel.
    ax, ay = leg_start if leg_start is not None else (x, y)
    sx, sy = target[0] - ax, target[1] - ay
    seg = math.hypot(sx, sy)
    if seg > 1e-6:
        ux, uy = sx / seg, sy / seg
        cross = (x - ax) * (-uy) + (y - ay) * ux
        corr = max(-MAX_CROSS_CORR, min(MAX_CROSS_CORR, K_CROSS * cross))
        aim = math.atan2(uy, ux) - corr
    else:
        aim = math.atan2(dy, dx)

    err = math.atan2(math.sin(aim - yaw), math.cos(aim - yaw))

    cmd = Twist()
    if abs(err) > ALIGN_TOL:
        # Turn in place first so every leg is driven as a straight line.
        cmd.angular.z = max(-L.W_MAX, min(L.W_MAX, K_ANG * err))
    else:
        cmd.linear.x = max(0.0, min(L.V_MAX, K_LIN * dist))
        cmd.angular.z = max(-L.W_MAX, min(L.W_MAX, K_ANG * err))
    return cmd, False


class Coverage:
    def __init__(self):
        rospy.init_node('house_full_coverage')

        self._model = rospy.get_param('~robot_model', 'turtlebot3')
        self._pts, self._marks = build_tour()
        self._i = 0
        self._pose = None
        self._done = False

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._room_pub = rospy.Publisher('/coverage_room', String,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._on_states,
                         queue_size=1)

        total = sum(1 for _ in self._pts)
        rospy.loginfo('Tour: %d waypoints over %d rooms.', total, len(L.TOUR))
        for name, a, b in self._marks:
            rospy.loginfo('  %s  waypoints %d..%d', L.ROOMS[name]['label'], a, b)

        self._room = None
        rospy.Timer(rospy.Duration(1.0 / RATE_HZ), self._tick)

    def _on_states(self, msg):
        try:
            i = msg.name.index(self._model)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (p.x, p.y, yaw)

    def _current_room(self):
        for name, a, b in self._marks:
            if a <= self._i <= b:
                return name
        return None

    def _tick(self, _event):
        if self._pose is None or self._done:
            return

        if self._i >= len(self._pts):
            self._finish()
            return

        room = self._current_room()
        if room != self._room:
            self._room = room
            if room:
                rospy.loginfo('--> %s', L.ROOMS[room]['label'])
                self._room_pub.publish(String(data=room))

        leg_start = self._pts[self._i - 1] if self._i > 0 else None
        cmd, reached = steer(self._pose, leg_start, self._pts[self._i])
        if reached:
            self._i += 1
            return
        self._pub.publish(cmd)

    def _finish(self):
        self._done = True
        self._pub.publish(Twist())
        rospy.loginfo('Tour complete - every room swept.')


def main():
    Coverage()
    rospy.spin()


if __name__ == '__main__':
    main()
