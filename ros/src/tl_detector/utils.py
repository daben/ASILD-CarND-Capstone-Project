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


class benchmark(object):
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


class Waypoints(object):
    def __init__(self, waypoints):
        self.xy = np.empty((len(waypoints), 2), np.float32)
        self.dx = np.empty(len(waypoints), np.float32)
        self.dy = np.empty(len(waypoints), np.float32)
        self.update(waypoints)

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
        return dist.argmin()

    def __getitem__(self, i):
        return self.waypoints[i]
