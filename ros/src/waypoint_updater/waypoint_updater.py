#!/usr/bin/env python

import rospy
import math

import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg     import Lane, Waypoint

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

LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION = 500
LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Some params
        self.decel_limit     = rospy.get_param('~decel_limit', -5.0)
        self.velocity        = rospy.get_param('~velocity', 5.0)

        # Subs
        rospy.Subscriber('/current_pose'     , PoseStamped , self.pose_cb)
        rospy.Subscriber('/base_waypoints'   , Lane        , self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Pubs
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Format of geometry_msgs/Pose
        self.current_pose = None

        # Format of styx_msgs/Waypoint[]
        self.base_waypoints = None
        self.base_waypoints_np = None

        # Keep track of latest index in waypoints
        # So we start the research from that point
        self.last_closest_waypoint_index = 0

        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints

        # copy each waypoint position in numpy array
        self.base_waypoints_np = np.zeros((len(self.base_waypoints),2), dtype=np.double)
        for i in range(len(self.base_waypoints)):
            self.base_waypoints_np[i, 0] = self.base_waypoints[i].pose.pose.position.x
            self.base_waypoints_np[i, 1] = self.base_waypoints[i].pose.pose.position.y

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

    def distance(self, waypoints, wp1_index, wp2_index):
        """Compute the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two.

        Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list.
        This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light (the velocities should gradually decrease to zero starting some distance from the light).
        """
        dist = 0
        dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2
        for i in range(wp1_index+1, wp2_index+1):
            dist += dl(waypoints[wp1_index].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return  math.sqrt(dist)

    def decelerate(self, waypoints, wp_stop_index):
        # All waypoints from stop should be zero
        for i in range(wp_stop_index, len(waypoints)):
            waypoints[i].twist.twist.linear.x = 0.0

        # Now decelerate to the stop position bases on max decel
        for i in range(wp_stop_index):
            #Get the distance till target
            dist = self.distance(waypoints, i, wp_stop_index)
            # Estimate a target velocity from the distance
            vel = math.sqrt(abs(self.decel_limit) * dist)
            if vel < 1.:
                vel = 0.
            waypoints[i].twist.twist.linear.x = min(vel, waypoints[i].twist.twist.linear.x)

    def get_closest_waypoint_index_np(self, waypoints, pose):
        '''
          Return the index of the closest waypoint from the pose

          waytpoints : numpy array [waypoints][0] = X , [waypoints][1] = Y
          pose       : geometry_msgs/Pose

        '''

        deltaX = waypoints[:,0] - pose.position.x
        deltaY = waypoints[:,1] - pose.position.y
        square_distance = (deltaX*deltaX) + (deltaY*deltaY)
        return square_distance.argmin()

    def generate_final_waypoints(self):
        '''
          Generate the final waypoints based on the base waypoints and our current
          pose.

          Get the closest waypoint from our current position and add up till LOOKAHEAD_WPS.
        '''
        idx = self.get_closest_waypoint_index_np(self.base_waypoints_np, self.current_pose)
        #rospy.loginfo("WP idx: {}".format(idx))

        waypoints = []

        # Handle wrapping
        if idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION <= len(self.base_waypoints):
            waypoints.extend(self.base_waypoints[idx:idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION])
        else:
            waypoints.extend(self.base_waypoints[idx:])
            waypoints.extend(self.base_waypoints[0:LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION - len(waypoints)])


        # Set a target speed based on current speed limit
        for i in range(LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION):
            waypoints[i].twist.twist.linear.x  = 10.0 #self.velocity


        # If need to stop
        if idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION >= 500:
            # TEST: Try to stop on waypoint 500
            if idx < 500:
                stop_index = 500 - idx - 1
                self.decelerate(waypoints, stop_index)
            else:
                for i in range(LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION):
                    waypoints[i].twist.twist.linear.x  = 0.0

        return waypoints[:LOOKAHEAD_WPS]

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # If we know our position, get the waypoints in front of us
            if self.current_pose is not None and self.base_waypoints_np is not None:
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.generate_final_waypoints()
                self.final_waypoints_pub.publish(lane)
            rate.sleep()



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
