#!/usr/bin/env python3
"""
coordinator_node - coverage accounting and live visualisation.

What counts as coverable
------------------------
A grid cell counts if its centre is (a) inside one of the room rectangles in
house_layout and (b) not inside a wall band or a solid block. That means the
denominator is exactly the reachable floor - wall interiors and the study's
solid block are excluded rather than being permanently-missing percent.

What counts as covered
----------------------
A cell is covered once the robot's swept band passes over it, i.e. its centre
comes within COVER_RADIUS of the robot centre. COVER_RADIUS models a cleaning
head wider than the chassis, which is what lets the sweep reach the strip
along each wall that the robot centre can never enter.

Publishes:
  /coverage_grid     nav_msgs/OccupancyGrid - 100 covered, 0 pending, -1 n/a
  /coverage_percent  std_msgs/Float32       - whole-house percentage
  /coverage_rooms    std_msgs/String        - "kitchen:100.0 bedroom:42.3 ..."
"""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String

from tb3_house_full_coverage import house_layout as L

LOG_PERIOD = 5.0
GRID_PERIOD = 1.0


class Coordinator:
    def __init__(self):
        rospy.init_node('house_full_coordinator')
        self._model = rospy.get_param('~robot_model', 'turtlebot3')

        self._res = L.RESOLUTION
        self._ox = L.X_MIN - L.MAP_PAD
        self._oy = L.Y_MIN - L.MAP_PAD
        self._w = int(round((L.X_MAX - L.X_MIN + 2 * L.MAP_PAD) / self._res))
        self._h = int(round((L.Y_MAX - L.Y_MIN + 2 * L.MAP_PAD) / self._res))

        # room index per cell (None = not coverable)
        self._room = [None] * (self._w * self._h)
        self._covered = bytearray(self._w * self._h)
        self._totals = {n: 0 for n in L.TOUR}
        self._done = {n: 0 for n in L.TOUR}

        for r in range(self._h):
            y = self._oy + (r + 0.5) * self._res
            for c in range(self._w):
                x = self._ox + (c + 0.5) * self._res
                if L.is_occupied(x, y):
                    continue
                name = L.room_at(x, y)
                if name is None:
                    continue
                self._room[r * self._w + c] = name
                self._totals[name] += 1

        self._grand = sum(self._totals.values())
        rospy.loginfo('Coverable floor: %d cells (%.2f m2) at %.2f m resolution',
                      self._grand, self._grand * self._res ** 2, self._res)
        for n in L.TOUR:
            rospy.loginfo('  %s %5d cells (%.2f m2)',
                          L.ROOMS[n]['label'], self._totals[n],
                          self._totals[n] * self._res ** 2)

        # Search radius in cells; the exact test below uses the robot's true
        # position, not its cell centre - snapping first loses up to half a
        # cell and wrongly fails cells right at the edge of the band.
        self._rad = int(math.ceil(L.COVER_RADIUS / self._res)) + 1

        self._grid_pub = rospy.Publisher('/coverage_grid', OccupancyGrid,
                                         queue_size=1, latch=True)
        self._pct_pub = rospy.Publisher('/coverage_percent', Float32,
                                        queue_size=1, latch=True)
        # "kitchen:100.0 bedroom:42.3 ..." - lets tb3_voice_coverage tell when
        # a requested room is finished.
        self._rooms_pub = rospy.Publisher('/coverage_rooms', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._on_states,
                         queue_size=1)

        self._reported_full = False
        rospy.Timer(rospy.Duration(LOG_PERIOD), self._log)
        rospy.Timer(rospy.Duration(GRID_PERIOD), self._publish_grid)

    def _on_states(self, msg):
        try:
            i = msg.name.index(self._model)
        except ValueError:
            return
        p = msg.pose[i].position
        rc = int((p.y - self._oy) / self._res)
        cc = int((p.x - self._ox) / self._res)
        rr = self._rad
        for r in range(max(0, rc - rr), min(self._h, rc + rr + 1)):
            cy = self._oy + (r + 0.5) * self._res
            for c in range(max(0, cc - rr), min(self._w, cc + rr + 1)):
                k = r * self._w + c
                name = self._room[k]
                if name is None or self._covered[k]:
                    continue
                cx = self._ox + (c + 0.5) * self._res
                if math.hypot(cx - p.x, cy - p.y) <= L.COVER_RADIUS:
                    self._covered[k] = 1
                    self._done[name] += 1

    def _total_pct(self):
        got = sum(self._done.values())
        return 100.0 * got / self._grand if self._grand else 0.0

    def _log(self, _event):
        for n in L.TOUR:
            t = self._totals[n]
            if not t:
                continue
            rospy.loginfo('%s %6.1f%%  (%d/%d cells)',
                          L.ROOMS[n]['label'], 100.0 * self._done[n] / t,
                          self._done[n], t)
        pct = self._total_pct()
        rospy.loginfo('WHOLE HOUSE  %6.2f%%  (%d/%d cells)',
                      pct, sum(self._done.values()), self._grand)
        self._pct_pub.publish(Float32(data=pct))
        self._rooms_pub.publish(String(data=' '.join(
            '%s:%.1f' % (n, 100.0 * self._done[n] / self._totals[n])
            for n in L.TOUR if self._totals[n])))

        if pct >= 100.0 and not self._reported_full:
            self._reported_full = True
            # No format args here, so '%' must NOT be escaped as '%%'.
            rospy.loginfo('=== 100% COVERAGE REACHED - every coverable cell swept ===')

    def _publish_grid(self, _event):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = rospy.Time.now()
        g.info.resolution = self._res
        g.info.width = self._w
        g.info.height = self._h
        g.info.origin.position.x = self._ox
        g.info.origin.position.y = self._oy
        g.info.origin.orientation.w = 1.0
        g.data = [(-1 if self._room[k] is None
                   else (100 if self._covered[k] else 0))
                  for k in range(self._w * self._h)]
        self._grid_pub.publish(g)


def main():
    Coordinator()
    rospy.spin()


if __name__ == '__main__':
    main()
