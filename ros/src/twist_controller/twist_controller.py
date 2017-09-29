from __future__ import division

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858  # kg/gal
ONE_MPH = 0.44704  # Miles/hour to meters/second


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.tare_vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]

        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.brake_deadband = kwargs["brake_deadband"]

        # wheels radius in meters (to compute torque)
        self.wheel_radius = kwargs["wheel_radius"]
        # in meters
        self.wheel_base = kwargs["wheel_base"]
        # steer_ratio is the ratio between the turn of the steering wheel
        # and the turn of the wheels (~15)
        self.steer_ratio = kwargs["steer_ratio"]
        # note that this can b bigger than one turn
        self.max_steer_angle = kwargs["max_steer_angle"]
        # control parameters
        self.control_rate = kwargs["control_rate"]
        self.control_period = 1. / self.control_rate

        # account for some passengers
        self.vehicle_mass = self.tare_vehicle_mass + 150
        self.fuel_mass = self.fuel_capacity * GAS_DENSITY
        # minimum speed threshold
        self.min_speed = 4 * ONE_MPH

        # current acceleration estimation
        self.accel_lpf = LowPassFilter(0.5, 0.02)
        # current fuel estimation
        self.fuel_lpf = LowPassFilter(60.0, 0.1)
        # current linear speed as reported
        self.current_linear_speed = 0

        # throttle controller
        self.accel_pid = PID(kp=0.3, ki=0.01, kd=0.0, min=0., max=1.)
        # steering controller
        self.yaw_controller = YawController(self.wheel_base,
                                            self.steer_ratio,
                                            self.min_speed,
                                            self.max_lat_accel,
                                            self.max_steer_angle)

    def reset(self):
        """Reset controller integrators"""
        self.accel_pid.reset_integrator()

    def linear_speed_cb(self, linear_speed):
        """Update current linear speed."""
        # Compute acceleration
        accel = self.control_rate * (linear_speed - self.current_linear_speed)
        # Smoothing to reduce the high noise of the derivatives
        self.accel_lpf.filt(accel)

        self.current_linear_speed = linear_speed

    def fuel_cb(self, fuel_level):
        """Update fuel level.

        Args:
            fuel_level (float): percent from 0 to 100
        """
        self.fuel_lpf.filt(fuel_level)

    def control(self, target_linear_speed, target_angular_speed,
                current_linear_speed, current_angular_speed):

        # Note: steering angles are counter clockwise, therefore:
        #  steer < 0: steer to the right
        #  steer > 0: steer to the left

        # TODO: catch a lag >= 10 times the control period and reset the integrators

        target_linear_speed = abs(target_linear_speed)

        speed_error = target_linear_speed - current_linear_speed
        # approximate the desired acceleration
        accel = 2 * speed_error
        # clamp it between the (safe) limits
        accel = max(self.decel_limit, min(accel, self.accel_limit))

        if accel >= 0:
            accel_error = accel - self.accel_lpf.get()
            throttle = self.accel_pid.step(accel_error, self.control_period)
        else:
            # forget integral error
            self.accel_pid.reset_integrator()
            throttle = 0.

        if accel < -self.brake_deadband or target_linear_speed < self.min_speed:
            # current vehicle mass
            vehicle_mass = (self.vehicle_mass
                           # account for fuel consumption (0 in the simulator)
                           + self.fuel_lpf.get() / 100. * self.fuel_mass)
            # Torque = F x radius = m * a * radius * sin(\theta)
            brake = abs(accel) * vehicle_mass * self.wheel_radius
        else:
            brake = 0.

        steer = self.yaw_controller.get_steering(target_linear_speed,
                                                 target_angular_speed,
                                                 current_linear_speed)

        steer = max(-self.max_steer_angle, min(steer, self.max_steer_angle))

        rospy.logdebug("twist_control: "
                       "linear_velocity=%.2f, "
                       "angular_velocity=%.2f, "
                       "current_velocity=%.2f, "
                       "current_angular_velocity=%.2f",
                       target_linear_speed, target_angular_speed,
                       current_linear_speed, current_angular_speed)
        rospy.logdebug("accel_pid: %s", self.accel_pid)

        return throttle, brake, steer
