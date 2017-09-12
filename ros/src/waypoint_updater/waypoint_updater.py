#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


from styx_msgs.msg import Lane, Waypoint
import sys
import numpy as np

import math
import threading

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # waypoints base list
        self.wpsBase = None
        # numpy array waypoint with positions [X,Y], to optimize time computation
        self.wpsXY = None
        # waypoints critical section
        self.wpsLock = threading.RLock()

        # Number added to the closest waypoint index, to be sure to be in front and not in back.
        # with the closest algorithm, the waypoint found could be in front or in back of the car
        self.numberInFrontOfClosestWaypointIndex = 1

        rospy.spin()

    def pose_cb(self, msg):
        if msg is None or self.wpsXY is None:
            return

        with self.wpsLock:
            wpDistance, wpIndex = self.closestWp2PoseNumpy(msg.pose)
            wpClosest = self.wpsBase[wpIndex]

            wpOut = self.selectWP(self.wpsBase, wpIndex)
            wpX = self.wpsXY[wpIndex, 0]
            wpY = self.wpsXY[wpIndex, 1]

        lane = Lane();
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = wpOut

        self.final_waypoints_pub.publish(lane)

        rospy.logdebug("final wp index:%d, dist:%.2f; x:%.2f, y:%.2f; cx:%.2f, cy:%.2f",
                       wpIndex, wpDistance,
                       wpClosest.pose.pose.position.x,
                       wpClosest.pose.pose.position.y,
                       msg.pose.position.x,
                       msg.pose.position.y)

    def waypoints_cb(self, msg):
        if msg is None:
            return

        waypoints = msg.waypoints

        # CAUTION. Don't overwrite self.wpsXY to avoid a race condition
        # copy each waypoint position in numpy array
        xy = np.zeros((len(waypoints), 2))
        for i, wp in enumerate(waypoints):
            xy[i, 0] = wp.pose.pose.position.x
            xy[i, 1] = wp.pose.pose.position.y

        with self.wpsLock:
            self.wpsBase = waypoints
            self.wpsXY = xy

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # todo, start form last position --> maybe to usefull with numpy computing
    def closestWp2PoseNumpy(self, pose):
        wpDeltaX = self.wpsXY[:, 0] - pose.position.x
        wpDeltaY = self.wpsXY[:, 1] - pose.position.y
        deltaSquareDistance = (wpDeltaX*wpDeltaX)+(wpDeltaY*wpDeltaY)

        minDistanceIndex = deltaSquareDistance.argmin()
        minDistance = deltaSquareDistance[minDistanceIndex]
        return minDistance, minDistanceIndex

    def selectWP(self, waypoints, indexClosest):
        idx = (indexClosest + self.numberInFrontOfClosestWaypointIndex) % len(waypoints)

        wpOut = waypoints[idx:idx+LOOKAHEAD_WPS]
        # while to avoid problem if len(waypoint)<LOOKAHEAD_WPS
        # make assumption than track looped
        while len(wpOut) < LOOKAHEAD_WPS:
            wpOut.extend(waypoints[:LOOKAHEAD_WPS - len(wpOut)])

        return wpOut


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
