
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Throttle in pourcentage
THROTTLE_MAX = 1.
THROTTLE_MIN = 0.
# Brake in Nm (torque)
BRAKE_MAX = 20000.
BRAKE_MIN = 0.

import rospy
import time

from pid            import PID
from yaw_controller import YawController
from lowpass        import LowPassFilter


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, sample_frequency):
        self.throttle_pid         = PID(0.2   , 0.0, 0.0, THROTTLE_MIN, THROTTLE_MAX)
        self.brake_pid            = PID(6000.0, 0.0, 0.0, BRAKE_MIN, BRAKE_MAX)
        self.steer_correction_pid = PID(0.17  , 0.0, 0.26, -1.0, 1.0) #0.3 0 0.4
        self.yaw_controller    = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        #self.yaw_lowpassfilter = LowPassFilter(1.,9.);
        self.ltime = None

        self.max_steer_angle = max_steer_angle


    def control(self, current_lvel, target_avel, target_lvel, cte):
        if current_lvel < 0.3 and target_lvel < 0.3:
            self.reset()
            return THROTTLE_MIN, BRAKE_MAX, 0.0

        ctime = time.time()
        th = 0.
        br = 0.
        st = 0.
        steer_angle = 0.

        if self.ltime is not None:
            v_error = target_lvel - current_lvel

            th =   self.throttle_pid.step(v_error, ctime - self.ltime)
            br = - self.brake_pid.step(v_error, ctime - self.ltime)
            st = - self.steer_correction_pid.step(cte, ctime - self.ltime)

            #th = min(THROTTLE_MAX, max(THROTTLE_MIN, th))
            #br = min(BRAKE_MAX   , max(BRAKE_MIN   , br))


            steer_angle = self.yaw_controller.get_steering(target_lvel, target_avel,current_lvel)
            steer_angle = min(self.max_steer_angle, max(-self.max_steer_angle, steer_angle + st))
            #steer_angle = self.yaw_lowpassfilter.filt(steer_angle)

            #rospy.loginfo('CL: {} - TL: {} - VE: {} - TH: {} - BR: {} - TT: {:f}'.format(current_lvel, target_lvel, target_lvel - current_lvel, th, br, ctime - self.ltime))



        self.ltime = ctime

        return th, br, steer_angle


    def reset(self):
        self.throttle_pid.reset_integrator()
        self.brake_pid.reset_integrator()
        self.steer_correction_pid.reset_integrator()
        self.ltime = None
