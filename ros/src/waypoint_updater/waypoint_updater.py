#!/usr/bin/env python
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math
import threading
from utils import benchmark
from utils import loop_at_rate
from utils import Waypoints
from utils import compute_crosstrack_error


# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 100


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        self.waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.tl_upcoming = None
        self.max_decel = rospy.get_param("~max_decel", 2.)

        self.final_waypoints_publisher = \
            rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.waypoints_subscriber = \
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.pose_subscriber = \
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.velocity_subscriber = \
            rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.traffic_subscriber = \
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.loop()

    def pose_cb(self, msg):
        """Callback for /current_pose."""
        self.current_pose = msg

    def velocity_cb(self, msg):
        self.current_velocity = msg

    def waypoints_cb(self, msg):
        """Callback for /base_waypoints."""
        if msg is not None:
            if self.waypoints is None:
                self.waypoints = Waypoints(msg.waypoints)
            else:
                self.waypoints.update(msg.waypoints)
            # NOTE. /base_waypoints are constant unsubscribe to save cpu
            self.waypoints_subscriber.unregister()

    def traffic_cb(self, msg):
        """Callback for /traffic_waypoint message."""
        self.tl_upcoming = msg.data
        # rospy.logdebug("tl_upcoming = %d", self.tl_upcoming)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def loop(self):
        publish_rate = rospy.get_param("~publish_rate", 50)

        while not rospy.is_shutdown():
            rospy.logwarn("waypoint updater waiting for data")
            for _ in loop_at_rate(publish_rate):
                # Wait for waypoints and pose and traffic lights
                if (self.waypoints and self.current_pose and
                    self.tl_upcoming is not None):
                    break

            rospy.loginfo("waypoint updater ready")
            for _ in loop_at_rate(publish_rate):
                # FIXME: This doesn't work with the bags
                # if (rospy.get_time() - self.current_pose.header.stamp.to_sec()) > 3:
                #     rospy.logwarn("current pose too old... cancelling")
                #     self.current_pose = None
                #     break

                with benchmark("update waypoints"):
                    waypoints = self.update_waypoints(self.current_pose,
                                                      self.tl_upcoming)

                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = waypoints

                self.final_waypoints_publisher.publish(lane)

    def update_waypoints(self, pose, tl_wp):
        car_x = pose.pose.position.x
        car_y = pose.pose.position.y
        car_wp = self.waypoints.find(car_x, car_y)

        waypoints = self.waypoints.slice(car_wp, min(len(self.waypoints), LOOKAHEAD_WPS))

        cte = compute_crosstrack_error(pose.pose, waypoints)
        # cte = float('nan')

        rospy.logdebug("tl_wp = %d; car_wp = %d", tl_wp, car_wp)
        if tl_wp >= car_wp:
            # NOTE. Give a margin to stop the car behind the stop line.
            # TODO. We should use the actual length of the car.
            tl_wp = (tl_wp - 2) % len(self.waypoints)
            self.decelerate(waypoints, car_wp, max(tl_wp, car_wp))
        else:
            tl_wp = -1
            # rospy.loginfo("upcoming traffic light in %d waypoints", tl_wp - car_wp)

        rospy.logdebug("ego wp=%d, dist=%.2f; x=%.2f, y=%.2f; car_x=%.2f, car_y=%.2f; speed=%.1f, wp=%.1f; tl=%s; cte=%.3f",
            car_wp, self.waypoints.distance_to_point(car_wp, car_x, car_y),
            self.waypoints[car_wp].pose.pose.position.x,
            self.waypoints[car_wp].pose.pose.position.y,
            pose.pose.position.x,
            pose.pose.position.y,
            self.current_velocity.twist.linear.x if self.current_velocity else float('nan'),
            waypoints[0].twist.twist.linear.x,
            str(tl_wp - car_wp) if tl_wp >= 0 else 'n/a',
            cte)
        rospy.logdebug("...(speeds: %s)", [round(wp.twist.twist.linear.x, 1) for wp in waypoints[:5]])

        return waypoints

    def decelerate(self, waypoints, start_index, stop_index):
        """Decelerate a list of waypoints until stop.

        Args:
            waypoints (list): waypoints to decelerate
            start (int): absolute index of the first waypoint in waypoints
            stop (int): absolute index of the waypoint where speed = 0
        """
        # relative index
        stop = stop_index - start_index

        if stop >= len(waypoints):
            end = len(waypoints)
            dist = self.waypoints.distance(start_index + end, stop_index)
        else:
            end = stop
            dist = 0.
            # All waypoints beyond stop should have speed 0
            for wp in waypoints[end:]:
                wp.twist.twist.linear.x = 0.

        if end == 0:
            return

        current_velocity = self.current_velocity.twist.linear.x

        prev_pos = self.waypoints[start_index + end].pose.pose.position

        # Decelerate the rest of the waypoints accelerating from the stop
        max_decel = self.max_decel
        for index in xrange(end - 1, -1, -1):
            wp = waypoints[index]
            wp_pos = wp.pose.pose.position
            # Compute distance
            dist += math.sqrt((wp_pos.x - prev_pos.x)**2 +
                              (wp_pos.y - prev_pos.y)**2 +
                              (wp_pos.z - prev_pos.z)**2)
            prev_pos = wp_pos
            # Adjust target speed
            vel = math.sqrt(2 * max_decel * dist)
            if vel < 1.: vel = 0.
            # if current_velocity > 1:
            #     vel = min(vel, current_velocity)
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
