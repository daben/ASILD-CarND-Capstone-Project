import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from styx_msgs.msg import Waypoint


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
            rospy.logdebug("benchmark '%s': %.3f secs", self.msg, self.elapsed)
        else:
            rospy.logdebug("benchmark '%s': %.0f msecs", self.msg, self.elapsed * 1e3)


benchmark = benchmark_disabled

def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.z * q.w + q.x * q.y),
                     - 1.0 + 2.0 * (q.w * q.w + q.x * q.x))

def compute_crosstrack_error(pose, waypoints):
    ego_x = pose.position.x
    ego_y = pose.position.y

    ego_yaw = yaw_from_quaternion(pose.orientation)
    cos_yaw = math.cos(-ego_yaw)
    sin_yaw = math.sin(-ego_yaw)

    # Convert to a frame relative to the pose
    wp_x = np.zeros(len(waypoints))
    wp_y = np.zeros(len(waypoints))
    for i, wp in enumerate(waypoints):
        dx = wp.pose.pose.position.x - ego_x
        dy = wp.pose.pose.position.y - ego_y
        wp_x[i] = dx * cos_yaw - dy * sin_yaw
        wp_y[i] = dx * sin_yaw + dy * cos_yaw

    # fit to a polynomial of degree 3
    poly = np.poly1d(np.polyfit(wp_x, wp_y, 3))

    # the crosstrack error is the independent coefficient
    cte = poly(0.)

    # for i, wp in enumerate(waypoints):
    #     wp.twist.twist.linear.y = poly(wp_x[i])

    return cte


class Waypoints(object):
    def __init__(self, waypoints):
        self.size = len(waypoints)
        self.xy = np.empty((len(waypoints), 2), np.float32)
        self.dx = np.empty(len(waypoints), np.float32)
        self.dy = np.empty(len(waypoints), np.float32)
        # initialize
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
        return dist.argmin()

    def __getitem__(self, idx):
        return self.waypoints[idx]

    def copy_waypoint(self, idx):
        ref_wp = self.waypoints[idx]
        waypoint = Waypoint()
        # Copy pose because is constant
        waypoint.pose = ref_wp.pose
        # ditto
        waypoint.twist.header = ref_wp.twist.header
        # This can be updated
        waypoint.twist.twist.linear.x = ref_wp.twist.twist.linear.x
        waypoint.twist.twist.linear.y = ref_wp.twist.twist.linear.y
        waypoint.twist.twist.linear.z = ref_wp.twist.twist.linear.z
        waypoint.twist.twist.angular.x = ref_wp.twist.twist.angular.x
        waypoint.twist.twist.angular.y = ref_wp.twist.twist.angular.y
        waypoint.twist.twist.angular.z = ref_wp.twist.twist.angular.z
        return waypoint

    def slice(self, offset, size):
        n = self.size
        if offset + size < n:
            return list(self.copy_waypoint(i)
                        for i in xrange(offset, offset+size))
        else:
            return list(self.copy_waypoint(i % n)
                        for i in xrange(offset, offset+size))

    def distance(self, wp1, wp2):
        if wp2 < wp1:
            return (self.distance(wp1, len(self)-1) +
                    self.distance(0, wp2))
        else:
            wps = self.waypoints
            dist = 0
            for i in xrange(wp1, wp2):
                a = wps[i+0].pose.pose.position
                b = wps[i+1].pose.pose.position
                dist += math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            return dist

    def distance_to_point(self, wp_i, x, y):
        wp = self.waypoints[wp_i]
        return math.sqrt((wp.pose.pose.position.x - x)**2 +
                         (wp.pose.pose.position.y - y)**2)
