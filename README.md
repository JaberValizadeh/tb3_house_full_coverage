# tb3_house_full_coverage

One **TurtleBot3** covering an entire house on its own, room after room, and
finishing at **100% of the reachable floor**.

Where `tb3_house_coverage` splits a house between four robots and tops out
around 94–96%, this package uses a single robot and closes that last few
percent: it sweeps each room's **edges first**, then fills the middle.

## The house

Six spaces, deliberately different shapes, so the sweep has to handle more
than plain rectangles:

```
+-----------------------------------------------+
|   KITCHEN     |  LIVING (L)  |    BATHROOM     |
|   rectangle   |        +-----+                 |
|               |        |STORE|                 |
|               |        |small|                 |
+===============+========+=====+=================+   <- doors
|                  CORRIDOR (long, narrow)       |
+===============+===============================+    <- doors
|    BEDROOM    |  STUDY (U-shaped, wraps a      |
|   rectangle   |  solid block)  +=======+       |
|               |                | BLOCK |       |
|               |                +=======+       |
+-----------------------------------------------+
```

| Space | Shape | Notes |
|---|---|---|
| Kitchen | rectangle | off the corridor |
| Living room | **L-shape** | swept as two overlapping rectangles |
| Store room | small square | reached **through the living room**, not the corridor |
| Bathroom | rectangle | off the corridor |
| Bedroom | rectangle | off the corridor |
| Study | **U-shape** | wraps a solid block the robot must go around |
| Corridor | long narrow strip | the transit space, swept last |

Tour order: kitchen → living → store → bathroom → bedroom → study → corridor.

## Run

Three stages, in three terminals. Nothing moves until stage 3.

```bash
# Terminal 1 - house world + robot
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch tb3_house_full_coverage environment.launch      # or gui:=false

# Terminal 2 - map + AMCL + RViz
source ~/catkin_ws/devel/setup.bash
source ~/nav_underlay/setup_nav.bash                      # nav stack (see below)
roslaunch tb3_house_full_coverage localization.launch     # rviz:=false to skip RViz

# Terminal 3 - coverage accounting + the sweep
source ~/catkin_ws/devel/setup.bash
roslaunch tb3_house_full_coverage coverage.launch
```

Watch progress: `rostopic echo /coverage_percent`, or the room-by-room
breakdown logged every 5 s. In RViz the `/coverage_grid` layer fills in behind
the robot. The full tour is ~144 m of driving, so allow roughly 11 minutes at
the Burger's 0.22 m/s.

`sweep:=false` brings up the coordinator alone (room breakdown + empty grid,
robot parked); start the sweep afterwards with `sweep.launch`.

| Launch file | Stage | Brings up |
|---|---|---|
| `environment.launch` | 1 | house world + one robot in the corridor |
| `localization.launch` | 2 | `map_server` + `amcl` + RViz |
| `coverage.launch` | 3 | `coordinator_node` (+ the sweep) |
| `sweep.launch` | 3 (split) | `coverage_node` — the only thing that moves the robot |

## How 100% is reached

Getting to exactly 100% is a matter of being precise about two definitions and
then making the geometry honour them.

**What counts.** A cell counts if its centre is inside a room and *not* inside
a wall or the solid block. Wall interiors and the block are excluded from the
denominator rather than sitting there as permanently-missing percent.

**What counts as covered.** A cell is covered when its centre comes within
`COVER_RADIUS` (0.20 m) of the robot centre — a 0.40 m cleaning head on a
0.21 m chassis, the way real vacuums use side brushes. This is what makes the
strip along each wall reachable at all: the robot *centre* can never enter it,
but the brush can.

Three geometric rules then guarantee full coverage, each verified before
running:

1. **Edges before middle.** Each rectangle gets one perimeter lap, then
   boustrophedon lanes. Without the lap, the strip between a lane's end and the
   wall is only touched by the discs at the lane ends, and cells there that sit
   between two lanes fall just outside the band — the classic boustrophedon
   corner gap. The lap covers the whole boundary strip continuously. The first
   and last lanes are then skipped, since they coincide with the lap.
2. **Per-edge insets.** How close a lane may run to an edge is *probed*, not
   assumed. Rectangle edges meet on wall centrelines, so clearing a wall needs
   half the wall thickness plus the chassis radius — but the study's arms are
   bounded by the block's *face*, and seam edges have no wall at all. A single
   uniform margin left an unreachable pocket beside the block.
3. **Overlapping seams.** Where an L or U room is split into rectangles, the
   split line is not a wall. If the pieces merely touched, the corner where a
   seam meets a real wall would fall outside both sweeps, so pieces overlap by
   `SEAM_LAP` into free floor of the same room.

Every leg of the tour is axis-aligned — diagonal shortcuts between lanes,
doors and bridges are replaced by a safe corner, because a diagonal can clip
the corner of a wall it passes.

### Not driving the same line twice

Reaching 100% is easy if you are willing to sweep everything repeatedly. The
first working planner drove **35% of its distance without cleaning anything** -
most visibly the corridor, which every room transit crosses long before the
corridor's own sweep is planned, so that sweep was almost pure repetition.

Three changes fixed it, and `scripts/verify_coverage.py` keeps them honest:

1. **The planner knows what is already swept.** `prune()` simulates the swept
   band while planning and drops any leg that would reveal no new floor.
   `build_tour(covered=...)` seeds that simulator with floor cleaned *before*
   this plan was made — the live grid, say — so a second request or a top-up
   pass plans around it instead of re-driving it; `tb3_voice_coverage` uses
   this and gets kitchen+bathroom-after-kitchen down from 46.0 m to 28.4 m.
   Coverage cannot suffer, because each rectangle's chosen sweep still has to
   cover every cell the seed does not already account for. The
   skip test is deliberately conservative: removing a waypoint also re-routes
   the *next* leg, so a leg is only dropped when the resulting shortcut still
   sweeps everything the original would have, and only when that shortcut is
   collision-free. Checking merely "this leg cleans nothing" silently loses
   cells - it cost 270 of them before the test was tightened.
2. **The tour follows the corridor, not the room list.** Rooms are visited in
   door order from west to east, so the robot crosses the house once instead
   of bouncing between opposite ends. `order_rooms(rooms, start)` generalises
   that to a robot that is *not* at the west end — it runs to the nearer end
   of the span of requested doors, then straight across, which is the exact
   optimum for doors on one line. From the spawn pose it reproduces `TOUR`
   exactly, so the whole-house tour is unchanged.
3. **Each rectangle's sweep is chosen, not assumed.** The perimeter lap exists
   to close corner gaps, but in a rectangle narrow enough for the lanes alone
   to reach both walls it is a second full-length run over floor the lanes
   already cover. That made the corridor and the study's bottom bar the two
   most-repeated strips in the house - four 6.6 m runs down a corridor that
   needs three. `_choose_sweep` now offers the lap, both lane directions and
   both traversal orders as *candidates*, simulates each, and takes the
   shortest one that still covers the whole rectangle.

| | path | m driven per m² |
|---|---|---|
| original | 198.2 m | 7.13 |
| + pruning and door order | 184.2 m | 6.63 |
| + chosen sweeps | **144.4 m** | **5.20** |

Coverage stayed at 11121/11121 with zero wall contact throughout, and every
room subset got shorter too (the study alone: 44.1 m → 30.0 m). What remains
is mostly genuine room-to-room transit, which no amount of planning removes;
the theoretical floor for a 0.40 m brush is 2.50 m/m².

**Cheapest-first only works because the check is exact.** A candidate is
admissible only if the simulator says it sweeps every still-dirty cell inside
the rectangle - the claim is never argued from geometry. Two things are
deliberately *not* credited when checking, both learned the hard way: the leg
*into* a sweep (it may be diagonal, and `manhattanize` will later replace any
diagonal with two axis-aligned legs sweeping a different band), and diagonal
legs generally, which are credited only with the discs at their two ends.

Two further ideas were tried and reverted, which is why they are not here:
placing lanes against the *coverable extent* rather than the rectangle, and
offering sparser lane counts as candidates. Both looked strictly better and
both measured worse - 154.5 m, and seven lost living-room cells - because
shortening lanes strands their far ends, which forces the perimeter lap back
on, and the lap costs more than the ends saved.

**Following the plan matters as much as the plan.** A geometrically perfect
path still misses cells if the robot does not drive it. The controller
therefore corrects **cross-track** error — how far it is from the planned
line — not just heading to the next waypoint. Without that, accepting each
waypoint within `POS_TOL` lets the next leg start that far off and run
parallel to the plan, and the first live run finished at 99.96%: five doorway
threshold cells, which are free floor only because a door interrupts a wall
and therefore sit at the extreme edge of the swept band, were lost to a few
centimetres of drift.

### Checking it, per request

```bash
python3 scripts/verify_coverage.py               # the whole house
python3 scripts/verify_coverage.py kitchen study # one request
python3 scripts/verify_subsets.py                # every room, every pair
```

`verify_subsets.py` matters because the guarantee is *per request*, not just
for the full tour. A subset transits different floor on the way, so it reaches
each room with a different set of cells already swept, and the chooser makes
different decisions as a result. A planner change can hold the full house at
100% and still break "clean the store room". All 29 combinations currently
pass with zero missed cells and zero wall contacts.

Both scripts assume perfect path following, so they are a necessary condition
for 100%, not a sufficient one — real tracking error is what the live run
tests.

## Layout is the single source of truth

`tb3_house_full_coverage/house_layout.py` holds the walls, blocks, room
rectangles and tour. The Gazebo world, the `map_server` map, the sweep path
and the coverage accounting are all derived from it, so they cannot drift
apart. After editing it, regenerate the world and map:

```bash
cd ~/catkin_ws/src/tb3_house_full_coverage
python3 scripts/generate_house_world.py
```

## Nodes and topics

| Node | Role |
|---|---|
| `coverage_node` | Plans the whole tour up front - choosing each rectangle's sweep against a simulator of what is already covered - then follows it: turn in place, drive straight, repeat. The only publisher of `cmd_vel`. |
| `coordinator_node` | Coverage accounting per room and for the house; publishes the live grid |

| Topic | Type | Meaning |
|---|---|---|
| `/coverage_percent` | `std_msgs/Float32` | whole-house percentage |
| `/coverage_grid` | `nav_msgs/OccupancyGrid` | 100 covered, 0 pending, −1 not coverable |
| `/coverage_room` | `std_msgs/String` | room currently being swept |

Poses come from Gazebo ground truth (`/gazebo/model_states`), as in the other
tb3 coverage packages. AMCL in stage 2 is real LiDAR localization shown on the
map, but it is not the controller's input.

## Navigation stack note

Stage 2 needs `map_server` and `amcl`. This machine has no system-wide
`ros-noetic-navigation` install (and no sudo), so the stack lives in a
user-space underlay at `~/nav_underlay/root`, activated by sourcing
`~/nav_underlay/setup_nav.bash`. On a normally provisioned machine
(`sudo apt install ros-noetic-map-server ros-noetic-amcl`), skip that line.

## Tuning

- Room shapes, walls, block, tour order: `house_layout.py` (re-run the
  generator afterwards)
- Brush width / lane spacing: `COVER_RADIUS` and `LANE_W` — keep
  `LANE_W <= 2 * COVER_RADIUS` or adjacent lanes stop overlapping
- Clearances: `MARGIN`, `ROBOT_RADIUS`, `SEAM_LAP`
- Speed: `V_MAX` (Burger tops out at 0.22 m/s)
