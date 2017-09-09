from __future__ import division
from __future__ import print_function

from math import atan, fabs


class YawController(object):
    """A controller to convert target linear and angular velocity to steering angles.
    """

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = abs(min_speed)
        self.max_lat_accel = abs(max_lat_accel)

        self.min_angle = -abs(max_steer_angle)
        self.max_angle = abs(max_steer_angle)


    def get_angle(self, radius):
        """Returns steering angle."""
        angle = atan(self.wheel_base / float(radius)) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        """Returns steering wheel angle."""

        # if fabs(angular_velocity) < 1e-6:
        #     return 0.
        # else:
        #     return self.get_angle(linear_velocity / angular_velocity)

        if fabs(linear_velocity) < 1e-6:
            return 0.
        else:
            angular_velocity = current_velocity / linear_velocity * angular_velocity

        if fabs(current_velocity) > 0.1:
            max_yaw_rate = fabs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(angular_velocity, max_yaw_rate))

        if fabs(angular_velocity) < 1e-6:
            return 0.
        else:
            current_velocity = max(current_velocity, self.min_speed)
            return self.get_angle(current_velocity / angular_velocity)
