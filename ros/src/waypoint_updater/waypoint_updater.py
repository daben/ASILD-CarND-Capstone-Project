#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from matplotlib.cbook import maxdict

from styx_msgs.msg import Lane, Waypoint
import sys

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.loginfo("waypoint_updater : pierre init")
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.wpsBase=[];

        self.lastIndex =-1
        self.pose_cbIndex=0
        rospy.spin()


    def pose_cb(self, msg):
        # sub sampling current pose
        self.pose_cbIndex = self.pose_cbIndex +1
        if self.pose_cbIndex%5!=0:
            return
        #rospy.loginfo("pierre_pose" + str(msg.pose.position.x)+","+str(msg.pose.position.y))
        # TODO: Implement
        rospy.loginfo("pierre_wp_cp %f,%f",msg.pose.position.x,msg.pose.position.y)
        #self.ratePose.sleep()


        maxDistance, maxDistanceIndex = self.closestWp2Pose(msg.pose,self.wpsBase)
        wpOut = self.selectWP(self.wpsBase,maxDistanceIndex)
        lane=Lane();
        lane.waypoints = wpOut

        self.final_waypoints_pub.publish(lane)
        rospy.loginfo("pierre_wp_updater" + " publish final, wp index:%d, dist:%f,x%f,y%f;cx%f,cy%f", maxDistanceIndex,maxDistance,
                      self.wpsBase[maxDistanceIndex].pose.pose.position.x,self.wpsBase[maxDistanceIndex].pose.pose.position.y,
                      msg.pose.position.x,msg.pose.position.y)

        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.wpsBase = waypoints.waypoints;
        self.base_wp_sub.unregister() # unsubcribe after the first data receive. It is always the same data
        #rospy.loginfo("pierre_wp" + "_startList" +"1"+ str((waypoints)))
        #for wp in waypoints.waypoints:
            #rospy.loginfo("pierre_wp %s %s %s %s", str(type(wp)), str(type(wp.pose)), str(type(wp.pose.pose)), str(type(wp.pose.pose.position)))
        #    rospy.loginfo("pierre_wp," + str(wp.pose.pose.position.x)+","+str(wp.pose.pose.position.y))
        pass

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

    # not efficient, I test for everyone each time. todo, start form last position
    def closestWp2Pose(self, pose, waypoints):
        dl = lambda a, b: ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 )
        maxDistanceIndex=-1
        maxDistance = sys.float_info.max
        for i in range(0, len(waypoints)): # just for test replace by 2000
            dist = dl(waypoints[i].pose.pose.position, pose.position)
            if(dist<maxDistance):
                maxDistance = dist
                maxDistanceIndex=i
        return maxDistance,maxDistanceIndex

    def selectWP(self,waypoints,indexClosest):
        wpOut = [];

        # remove 5 in front to be sure to be infront and not back (the closest)
        idx = indexClosest + 5
        if(idx>len(waypoints)):
            idx = idx-len(waypoints)
        if(indexClosest+LOOKAHEAD_WPS>len(waypoints)):
            wpOut.extend(waypoints[idx:])
            wpOut.extend(waypoints[0:LOOKAHEAD_WPS-len(wpOut)])

        else:
            wpOut.extend(waypoints[indexClosest:indexClosest+LOOKAHEAD_WPS])

        return wpOut


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
