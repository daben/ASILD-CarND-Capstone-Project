import rospy
import time
import numpy as np


class loop_at_rate(object):
    def __init__(self, hz, **kwargs):
        self.rate = rospy.Rate(hz, **kwargs)

    def __iter__(self):
        while not rospy.is_shutdown():
            yield self.rate
            self.rate.sleep()


class benchmark_disabled(object):
    def __init__(self, *args): pass
    def __enter__(self): pass
    def __exit__(self, *args): pass


class benchmark_enabled(object):
    def __init__(self, msg):
        self.msg = msg
        self.start = 0
        self.elapsed = 0

    def __enter__(self):
        self.start = time.clock()

    def __exit__(self, *args):
        self.elapsed = time.clock() - self.start
        if self.elapsed > 1.:
            rospy.logdebug("%s: %.3f secs", self.msg, self.elapsed)
        else:
            rospy.logdebug("%s: %.0f msecs", self.msg, self.elapsed * 1e3)


benchmark = benchmark_enabled


class Waypoints(object):
    def __init__(self, waypoints):
        self.size = len(waypoints)
        self.xy = np.empty((self.size, 2), np.float32)
        self.dx = np.empty(self.size, np.float32)
        self.dy = np.empty(self.size, np.float32)
        self.update(waypoints)

    def __len__(self):
        return self.size

    def update(self, waypoints):
        self.waypoints = waypoints
        xy = self.xy
        for i, wp in enumerate(waypoints):
            xy[i, 0] = wp.pose.pose.position.x
            xy[i, 1] = wp.pose.pose.position.y

    def find(self, x, y):
        dx = np.subtract(self.xy[:, 0], x, self.dx)
        dy = np.subtract(self.xy[:, 1], y, self.dy)
        dx = np.multiply(dx, dx, dx)
        dy = np.multiply(dy, dy, dy)
        dist = np.add(dx, dy, dx)

        wp1 = dist.argmin()
        wp2 = (wp1 + 1) % len(self)

        # wp1 -> wp2
        dx1 = self[wp2].pose.pose.position.x - self[wp1].pose.pose.position.x
        dy1 = self[wp2].pose.pose.position.y - self[wp1].pose.pose.position.y
        # (x,y) -> wp1
        dx2 = self[wp1].pose.pose.position.x - x
        dy2 = self[wp1].pose.pose.position.y - y
        # sign of angle between wp1:wp2 and xy:wp1
        sign_a = (dx1 * dx2 + dy1 * dy2)

        return wp2 if sign_a >= 0 else wp1

    def __getitem__(self, i):
        return self.waypoints[i]
