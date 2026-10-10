"""
Single source of truth for the house: walls, solid blocks, room shapes and
the tour that visits them.

Everything else is derived from this module - the Gazebo world, the
occupancy map, the sweep path and the coverage accounting - so the
simulation, the map and the "100%" number can never disagree.

Layout (x in [-3.5, 3.5], y in [-2.5, 2.5]), six spaces of deliberately
different shapes:

    +-----------------------------------------------+
    |   KITCHEN     |  LIVING (L)  |    BATHROOM     |
    |   rectangle   |        +-----+                 |
    |               |        |STORE|                 |
    |               |        |small|                 |
    +===============+========+=====+=================+   <- doors
    |                  CORRIDOR (long, narrow)       |
    +===============+===============================+   <- doors
    |    BEDROOM    |  STUDY (U-shaped, wraps a      |
    |   rectangle   |  solid block)  +=======+       |
    |               |                | BLOCK |       |
    |               |                +=======+       |
    +-----------------------------------------------+

Shapes covered: rectangle (kitchen, bathroom, bedroom), L-shape (living),
small square (store), long narrow strip (corridor) and a U-shape wrapping
a solid obstacle (study).
"""

# ── Geometry ─────────────────────────────────────────────────────────────────
X_MIN, X_MAX = -3.5, 3.5
Y_MIN, Y_MAX = -2.5, 2.5

WALL_THICK = 0.15
WALL_H = 0.5

RESOLUTION = 0.05     # occupancy map / coverage grid resolution (m/px)
MAP_PAD = 0.2         # padding around the house in the map (m)

# ── Robot / sweep constants ──────────────────────────────────────────────────
# Room rectangles meet on wall CENTRELINES, so a lane inset from a rectangle
# edge must clear half the wall thickness before it clears the chassis:
#   MARGIN >= WALL_THICK/2 (0.075) + chassis radius (0.105) + clearance
# Getting this wrong drives the robot into the walls it is sweeping along.
ROBOT_RADIUS = 0.105
MARGIN = 0.21         # robot centre inset from a rectangle edge (m)

# COVER_RADIUS models a cleaning head wider than the chassis (a 0.40 m brush
# on a 0.21 m robot - exactly how real vacuums use side brushes). It is what
# makes 100% reachable: the closest a lane may run to a rectangle edge is
# MARGIN, and the nearest coverable cell centre sits WALL_THICK/2 from that
# edge, so the band reaches the wall face as long as
#   COVER_RADIUS >= MARGIN - WALL_THICK/2  (0.21 - 0.075 = 0.135).
COVER_RADIUS = 0.20   # half-width of the swept band (m)

# Adjacent lanes must overlap: LANE_W <= 2 * COVER_RADIUS (0.40), kept a
# little under so neighbouring bands genuinely overlap rather than just touch.
LANE_W = 0.38         # spacing between adjacent lanes (m)

# How far rectangles of one room overlap across a seam (see ROOMS below).
SEAM_LAP = 0.25

V_MAX = 0.22          # Burger top speed (m/s)
W_MAX = 1.8           # turn rate cap (rad/s)

SPAWN = (-3.0, 0.0, 0.0)   # x, y, yaw - west end of the corridor

# ── Walls: axis-aligned segments (x0, y0, x1, y1); gaps are doorways ─────────
WALLS = [
    # outer shell
    (X_MIN, Y_MIN, X_MAX, Y_MIN),
    (X_MIN, Y_MAX, X_MAX, Y_MAX),
    (X_MIN, Y_MIN, X_MIN, Y_MAX),
    (X_MAX, Y_MIN, X_MAX, Y_MAX),

    # corridor north wall y=0.45, doors: kitchen x[-2.6,-1.8],
    # living x[-0.5,0.3], bathroom x[2.0,2.8]
    (X_MIN, 0.45, -2.6, 0.45),
    (-1.8, 0.45, -0.5, 0.45),
    (0.3, 0.45, 2.0, 0.45),
    (2.8, 0.45, X_MAX, 0.45),

    # corridor south wall y=-0.45, doors: bedroom x[-2.5,-1.7], study x[-0.2,0.5]
    (X_MIN, -0.45, -2.5, -0.45),
    (-1.7, -0.45, -0.2, -0.45),
    (0.5, -0.45, X_MAX, -0.45),

    # north internal walls
    (-1.0, 0.45, -1.0, Y_MAX),      # kitchen | living
    (1.2, 0.45, 1.2, Y_MAX),        # living/store | bathroom
    (0.3, 1.5, 0.3, 1.75),          # living | store, door gap y[1.75,2.25]
    (0.3, 2.25, 0.3, Y_MAX),
    (0.3, 1.5, 1.2, 1.5),           # store floor wall

    # south internal wall
    (-0.5, Y_MIN, -0.5, -0.45),     # bedroom | study
]

# ── Solid blocks: filled boxes (x0, y0, x1, y1), not enterable ───────────────
# The block hangs off the corridor's south wall and is what makes the study
# U-shaped: the robot must go around it.
BLOCKS = [
    (0.6, -1.6, 2.4, -0.45),
]

# ── Rooms ────────────────────────────────────────────────────────────────────
# Each room is a list of axis-aligned rectangles (x0, y0, x1, y1) in the order
# they are swept, plus:
#   access  - waypoints from the corridor centreline to a point inside the room
#   bridges - one waypoint per pair of consecutive rects, on their shared edge,
#             so moving between rects never clips a wall
# The rectangles cover the room exactly, so the union of all rooms is the whole
# reachable floor.
#
# SEAMS: where an L- or U-shaped room is split, the split line is not a wall.
# Each rectangle insets its sweep by MARGIN from its own edges, so if two
# rectangles merely touched along a seam, the corner where that seam meets a
# real wall would fall outside both sweeps. Rectangles are therefore OVERLAPPED
# by SEAM_LAP across seams (into free floor of the same room), which costs a
# little repeated driving and closes the corner. Overlap within one room is
# harmless for the accounting - each cell is assigned to its room once.
ROOMS = {
    'kitchen': {
        'label': 'Kitchen      (rectangle)',
        'rects': [(X_MIN, 0.45, -1.0, Y_MAX)],
        'access': [(-2.2, 0.0), (-2.2, 0.8)],
        'bridges': [],
    },
    'living': {
        'label': 'Living room  (L-shape) ',
        # upper leg overlaps SEAM_LAP down into the lower one (free floor)
        'rects': [(-1.0, 0.45, 1.2, 1.5), (-1.0, 1.5 - SEAM_LAP, 0.3, Y_MAX)],
        'access': [(-0.1, 0.0), (-0.1, 0.8)],
        'bridges': [(-0.35, 1.5)],
    },
    'store': {
        'label': 'Store room   (small)   ',
        'rects': [(0.3, 1.5, 1.2, Y_MAX)],
        # reached through the living room, not off the corridor
        'access': [(-0.1, 0.0), (-0.1, 0.8), (-0.1, 2.0), (0.75, 2.0)],
        'bridges': [],
    },
    'bathroom': {
        'label': 'Bathroom     (rectangle)',
        'rects': [(1.2, 0.45, X_MAX, Y_MAX)],
        'access': [(2.4, 0.0), (2.4, 0.8)],
        'bridges': [],
    },
    'bedroom': {
        'label': 'Bedroom      (rectangle)',
        'rects': [(X_MIN, Y_MIN, -0.5, -0.45)],
        'access': [(-2.1, 0.0), (-2.1, -0.8)],
        'bridges': [],
    },
    'study': {
        'label': 'Study        (U-shape) ',
        # left arm -> bottom bar -> right arm, wrapping the solid block.
        # The bottom bar starts at the bedroom wall (x=-0.5), not the shell.
        # Both arms overlap SEAM_LAP down into the bottom bar; the bar itself
        # cannot be extended upward because the solid block is in the way.
        'rects': [(-0.5, -1.6 - SEAM_LAP, 0.6, -0.45),
                  (-0.5, Y_MIN, X_MAX, -1.6),
                  (2.4, -1.6 - SEAM_LAP, X_MAX, -0.45)],
        'access': [(0.15, 0.0), (0.15, -0.8)],
        'bridges': [(0.05, -1.6), (2.95, -1.6)],
    },
    'corridor': {
        'label': 'Corridor     (long)    ',
        'rects': [(X_MIN, -0.45, X_MAX, 0.45)],
        'access': [],          # the corridor IS the transit space
        'bridges': [],
    },
}

# Ordered by where each room's door sits along the corridor, west to east
# (kitchen -2.2, bedroom -2.1, living -0.1, store via living, study 0.15,
# bathroom 2.4), so the tour crosses the house once instead of zig-zagging
# back and forth between opposite ends.
#
# The corridor is swept last: it is the transit space every other room is
# reached through, so by the time its turn comes it is largely done already.
TOUR = ['kitchen', 'bedroom', 'living', 'store', 'study', 'bathroom', 'corridor']


# ── Helpers shared by the world generator, planner and coordinator ───────────
def is_occupied(x: float, y: float) -> bool:
    """True if (x, y) falls inside a wall band or a solid block."""
    half = WALL_THICK / 2.0
    for x0, y0, x1, y1 in WALLS:
        if (min(x0, x1) - half <= x <= max(x0, x1) + half and
                min(y0, y1) - half <= y <= max(y0, y1) + half):
            return True
    for x0, y0, x1, y1 in BLOCKS:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


def room_at(x: float, y: float):
    """Name of the room whose rectangles contain (x, y), else None."""
    for name in TOUR:
        for x0, y0, x1, y1 in ROOMS[name]['rects']:
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
    return None
