
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import rospy  # just log to debug


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement


        self.vehicle_mass = 1736.35
        self.brake_deadband = .1
        self.decel_limit = -5.0
        self.accel_limit = 1.0
        self.steer_ratio = 14.8
        self.max_lat_accel = 3.0
        self.max_steer_angle = 8.0
        self.wheel_base = 2.8498
        self.sampleFrequency = 50.0

        for key, value in kwargs.iteritems():
            if "vehicle_mass" == key:
                self.vehicle_mass = value
            if "brake_deadband" == key:
                self.brake_deadband = value
            if "decel_limit" == key:
                self.decel_limit = value
            if "accel_limit" == key:
                self.accel_limit = value
            if "steer_ratio" == key:
                self.steer_ratio = value
            if "max_lat_accel" == key:
                self.max_lat_accel = value
            if "max_steer_angle" == key:
                self.max_steer_angle = value
            if "wheel_base" == key:
                self.wheel_base = value
            if "sampleFrequency" == key:
                self.sampleFrequency = value


        self.yawController = YawController(self.wheel_base,self.steer_ratio,0.1,self.max_lat_accel,self.max_steer_angle)
        self.yawLowPassFilter = LowPassFilter(0.01,1.0/self.sampleFrequency);
        self.linearVelocityPID = PID(0.5,0.0,0.0,self.decel_limit,self.accel_limit)

        pass

    def control(self, wantedLinearVelocity, wantedAngularVelocity, currentLinearVelocity):
        # linear_velocity, angular_velocity, current_velocity):
        rawSteer=self.yawController.get_steering(wantedLinearVelocity, wantedAngularVelocity, currentLinearVelocity)
        steer = self.yawLowPassFilter.filt(rawSteer)
        linear = self.linearVelocityPID.step(wantedLinearVelocity-currentLinearVelocity,self.sampleFrequency)

        rospy.loginfo("pierre dbw CMD pid:error %f,out %f, rawSteer %f",wantedLinearVelocity-currentLinearVelocity,linear,rawSteer)

        if(linear>0.0):
            brake=0.0
            throttle = linear # not seem to percent by m/s-2/self.accel_limit # percent or 0->1 ?
        else:
            brake = (-linear + self.brake_deadband)*4000
            throttle = 0.0


        return throttle, brake, steer
        #return 1., 0., 0.

    def reset(self):
        self.linearVelocityPID.reset();
        rospy.loginfo("pierre dbw CMD reset PID")
