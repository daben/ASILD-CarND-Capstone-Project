#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


from styx_msgs.msg import Lane, Waypoint
import sys
import numpy as np

import math

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

        # TODO: Add other member variables you need below
        # waypoints base list
        self.wpsBase = None
        self.wpsPositions = None #numpy array waypoint positions [X,Y], to optimize time computation

        # Number added to the closest waypoint index, to be sure to be in front and not in back.
        # with the closest algorithm, the waypoint found could be in front or in back of the car
        self.numberInFrontOfClosestWaypointIndex = 1

        rospy.spin()

    def pose_cb(self, msg):
        if msg is None or self.wpsPositions is None:
            return

        minDistance, minDistanceIndex = self.closestWp2PoseNumpy(msg.pose)
        wpOut = self.selectWP(self.wpsBase, minDistanceIndex)
        lane = Lane();
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = wpOut

        self.final_waypoints_pub.publish(lane)
        rospy.logdebug("final wp index:%d, dist:%.2f; x:%.2f, y:%.2f; cx:%.2f, cy:%.2f",
                       minDistanceIndex, minDistance,
                       self.wpsBase[minDistanceIndex].pose.pose.position.x,
                       self.wpsBase[minDistanceIndex].pose.pose.position.y,
                       msg.pose.position.x,
                       msg.pose.position.y)

    def waypoints_cb(self, waypoints):
        if(waypoints is None):
            return
        self.wpsBase = waypoints.waypoints; # store waypoints as list
        # cancel it, bad idea !
        # self.base_wp_sub.unregister() # unsubcribe after the first data receive. It is always the same data

        # copy each waypoint position in numpy array
        self.wpsPositions = np.zeros((len(self.wpsBase),2), dtype=np.double)
        for i in range(0, len(self.wpsBase)):
            self.wpsPositions[i, 0] = waypoints.waypoints[i].pose.pose.position.x
            self.wpsPositions[i, 1] = waypoints.waypoints[i].pose.pose.position.y

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
        wpDeltaX = self.wpsPositions[:, 0] - pose.position.x
        wpDeltaY = self.wpsPositions[:, 1] - pose.position.y
        deltaSquareDistance = (wpDeltaX*wpDeltaX)+(wpDeltaY*wpDeltaY)

        minDistanceIndex = deltaSquareDistance.argmin()
        minDistance = deltaSquareDistance[minDistanceIndex]
        return minDistance, minDistanceIndex

    def selectWP(self,waypoints,indexClosest):
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
