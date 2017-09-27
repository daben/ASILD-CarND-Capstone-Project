#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

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

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)  # m/s^2
        accel_limit = rospy.get_param('~accel_limit', 1.)  # m/s^2
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.max_throttle = rospy.get_param('~max_throttle', 1.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(
                vehicle_mass=vehicle_mass,
                fuel_capacity=fuel_capacity,
                brake_deadband=brake_deadband,
                decel_limit=decel_limit,
                accel_limit=accel_limit,
                wheel_radius=wheel_radius,
                wheel_base=wheel_base,
                steer_ratio=steer_ratio,
                max_lat_accel=max_lat_accel,
                max_steer_angle=max_steer_angle,
                control_rate=50.)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        # rospy.Subscriber('/vehicle/steering_report', SteeringReport,
        #                  self.steering_report_cb)
        # rospy.Subscriber('/vehicle/throttle_report', Float32,
        #                  self.throttle_report_cb)
        # rospy.Subscriber('/vehicle/brake_report', Float32,
        #                  self.brake_report_cb)

        # If a safety driver does take over, the PID controller will
        # mistakenly accumulate error, so you will need to be mindful of
        # the DBW status
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.dbw_enabled = False

        self.current_velocity = None
        self.twist_cmd = None

        self.loop()

    def dbw_enabled_cb(self, msg):
        if msg.data != self.dbw_enabled:
            self.dbw_enabled = msg.data
            if self.dbw_enabled:
                self.controller.reset()
            rospy.logwarn("DBW status: %s", self.dbw_enabled)

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist
        self.controller.linear_speed_cb(self.current_velocity.linear.x)

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg.twist

    def brake_report_cb(self, msg):
        pass

    def throttle_report_cb(self, msg):
        pass

    def steering_report_cb(self, msg):
        # msg.steering_wheel_angle_cmd # steer angle radians
        # msg.speed  # linear speed in m/s
        pass

    def is_ready(self):
        return (self.current_velocity is not None
                and self.twist_cmd is not None)

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Only publish the control commands if dbw is enabled
            if self.dbw_enabled:

                if self.is_ready():
                    # Get predicted throttle, brake, and steering using `twist_controller`
                    throttle, brake, steering = \
                        self.controller.control(
                            self.twist_cmd.linear.x,
                            self.twist_cmd.angular.z,
                            self.current_velocity.linear.x,
                            self.current_velocity.angular.z)

                    rospy.logdebug(
                        "speed: %.1f / %.1f MPH, angular_speed: %.2f"
                        ", throttle: %.2f, brake: %.2f, steer: %.2f",
                        self.current_velocity.linear.x / 0.44704,
                        self.twist_cmd.linear.x / 0.44704,
                        self.twist_cmd.angular.z,
                        throttle, brake, steering)
                else:
                    throttle = brake = steering = 0.

                self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        if brake > 0:
            throttle = 0.

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = min(throttle, self.max_throttle)
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        # steering angle in radians -8 to 8
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)



if __name__ == '__main__':
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start DBW node.')
