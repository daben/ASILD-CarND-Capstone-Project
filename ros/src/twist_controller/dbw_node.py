#!/usr/bin/env python

import rospy
import math

from std_msgs.msg      import Bool
from dbw_mkz_msgs.msg  import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller  import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

# Expected frequency 50Hz
SAMPLE_FREQUENCY = 50


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        wheel_radius    = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd',
                                            SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd',
                                            BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle, SAMPLE_FREQUENCY)

        # Subs
        rospy.Subscriber('/twist_cmd'            , TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity'     , TwistStamped, self.current_vel_cb)
        rospy.Subscriber('/vehicle/dbw_enabled'  , Bool        , self.dbw_enabled_cb)

        # Format of geometry_msgs/Twist
        self.twist = None
        # Format of geometry_msgs/Twist
        self.current_velocity = None
        # Format of std_msgs/Bool
        self.is_dbw_enabled = False

        self.loop()

    def twist_cb(self, msg):
        self.twist = msg.twist

    def current_vel_cb(self, msg):
        self.current_velocity = msg.twist

    def dbw_enabled_cb(self, msg):
        # Reset controller if changing mode
        if self.is_dbw_enabled != msg.data:
            self.controller.reset()

        self.is_dbw_enabled = msg.data

    def loop(self):
        rate = rospy.Rate(SAMPLE_FREQUENCY)
        while not rospy.is_shutdown():
            if self.current_velocity is not None and self.twist is not None:
              if self.is_dbw_enabled:
                  throttle, brake, steering = self.controller.control(self.current_velocity.linear.x,
                                                                      self.twist.angular.z,
                                                                      self.twist.linear.x,
                                                                      self.twist.linear.z
                                                                      )
                  self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        if brake <= self.brake_deadband:
            brake = 0.0

        # Activate one at a time
        if throttle > 0.0:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
        else:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)



        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)



if __name__ == '__main__':
    DBWNode()
