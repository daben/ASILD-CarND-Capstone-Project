#!/usr/bin/env python

import rospy
import math
import threading

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

        # To set a critical section
        self.waypoint_lock = threading.RLock()

        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):

        # copy each waypoint position in numpy array
        base_waypoints_np = np.zeros((len(msg.waypoints),2), dtype=np.double)
        for i in range(len(msg.waypoints)):
            base_waypoints_np[i, 0] = msg.waypoints[i].pose.pose.position.x
            base_waypoints_np[i, 1] = msg.waypoints[i].pose.pose.position.y

        # This is to avoid concurent access to data which lead to
        # non consistent output, use a critical section
        if self.waypoint_lock.acquire(True):
            self.base_waypoints = msg.waypoints
            self.base_waypoints_np = base_waypoints_np
            self.waypoint_lock.release()

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

    def get_closest_waypoint_index(self, waypoints, pose):
        '''
          Return the index of the closest waypoint from the pose

          waytpoints : numpy array [waypoints][0] = X , [waypoints][1] = Y
          pose       : geometry_msgs/Pose

        '''

        deltaX = waypoints[:,0] - pose.position.x
        deltaY = waypoints[:,1] - pose.position.y
        square_distance = (deltaX*deltaX) + (deltaY*deltaY)
        return square_distance.argmin()


    def cte_to_segment(self, wp1, wp2, pose):
        '''
          Compute the cte from pos to line (wp1,wp2)
          wp1,wp2 : styx_msgs/Waypoint
                    wp2 should be after wp1 to get the correct cte sign
          pose : Format of geometry_msgs/Pose


          cte =
        '''
        v0 = [ wp2.pose.pose.position.x - wp1.pose.pose.position.x
             , wp2.pose.pose.position.y - wp1.pose.pose.position.y
             ]
        p0 = [ wp1.pose.pose.position.x
             , wp1.pose.pose.position.y
             ]
        p1 = [ pose.position.x
             , pose.position.y
             ]
        p = self.calculate_relative_coordinate(v0, p0, p1)

        # Cte is the y coord of our relative to v0
        return p[1]

    def calculate_relative_coordinate(self, v0, p0, p1):
        '''
          Calc the relative coordinate of p1 in the base v0 and ortho vector at v0
          on point p0
          v0 : array of coord [x,y]
          p1 : array of coord [x,y]
        '''

        # Base change to vector v0 on p0

        # Translation
        p = [ p1[0] - p0[0]
            , p1[1] - p0[1]
            ]

        # Compute cos/sin theta from world coord to v0 base
        norm = math.sqrt(v0[0]**2 + v0[1]**2)

        v0[0] /= norm
        v0[1] /= norm

        # Use the projection  |V| * costheta = Vx
        # But |V| = 1. => costheta = Vx
        costheta = v0[0]

        # Use the projection  |V| * sintheta = Vy
        # But |V| = 1. => sintheta = Vy
        sintheta = v0[1]

        # Now we can rotate and transform our position
        # x' =  x * costheta + y * sintheta
        # y' = -x * sintheta + y * costheta

        return [   p[0] * costheta + p[1] * sintheta
                , -p[0] * sintheta + p[1] * costheta
               ]

    def generate_final_waypoints(self):
        '''
          Generate the final waypoints based on the base waypoints and our current
          pose.

          Get the closest waypoint from our current position and add up till LOOKAHEAD_WPS.
        '''

        waypoints = []

        # Use a critical section, so we get no updates of waypoint during this time
        if self.waypoint_lock.acquire(True):

            idx = self.get_closest_waypoint_index(self.base_waypoints_np, self.current_pose)
            #rospy.loginfo("WP idx: {}".format(idx))

            # Make sure we get the point in front
            i1 = (idx + 1) % len(self.base_waypoints)
            v0 = [ self.base_waypoints[i1].pose.pose.position.x - self.base_waypoints[idx].pose.pose.position.x
                 , self.base_waypoints[i1].pose.pose.position.y - self.base_waypoints[idx].pose.pose.position.y
                 ]
            p0 = [ self.base_waypoints[idx].pose.pose.position.x
                 , self.base_waypoints[idx].pose.pose.position.y
                 ]
            p1 = [ self.current_pose.position.x
                 , self.current_pose.position.y
                 ]
            p = self.calculate_relative_coordinate(v0, p0, p1)

            # If positive x => meaning i am in front of the waypoint
            if p[0] > 0:
               idx  = i1

            # Compute the CTE, knowing that idx is in front
            i0 = idx - 1
            cte = self.cte_to_segment(self.base_waypoints[i0], self.base_waypoints[idx], self.current_pose)

            #rospy.loginfo('WP : idx {} - cte {}'.format(idx,cte))

            # Handle wrapping
            if idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION <= len(self.base_waypoints):
                waypoints.extend(self.base_waypoints[idx:idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION])
            else:
                waypoints.extend(self.base_waypoints[idx:])
                waypoints.extend(self.base_waypoints[0:LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION - len(waypoints)])


            # Set a target speed based on current speed limit
            for i in range(LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION):
                waypoints[i].twist.twist.linear.x  = 10.0 #self.velocity
                waypoints[i].twist.twist.linear.z  = cte

            # If need to stop
            #if idx+LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION >= 500:
            #    # TEST: Try to stop on waypoint 500
            #    if idx < 500:
            #        stop_index = 500 - idx - 1
            #        self.decelerate(waypoints, stop_index)
            #    else:
            #        for i in range(LOOKAHEAD_WPS_TO_ESTIMATE_DECELERATION):
            #            waypoints[i].twist.twist.linear.x  = 0.0
            self.waypoint_lock.release()
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
