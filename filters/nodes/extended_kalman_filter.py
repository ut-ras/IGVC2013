#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy

import pylab
import math
import random
import numpy
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from ocean_server_imu.msg import RawData
from filters.msg import RotatedIMUData,EKFData


OCEAN_SERVER_IMU_INDEX = 0
ENCODERS_INDEX = 1
GPS_INDEX = 2
SPHERO_IMU_INDEX = 3


kf = None


def oceanserver_imu_callback(data):
    # rospy.loginfo("Measure:\n%f\n%f", data.acceleration.x, data.acceleration.y)

    measurement_vector = numpy.matrix(
                            [ [data.acceleration.x],
                              [data.acceleration.y],
                              [data.roll],
                              [data.pitch],
                              [data.yaw] ] )
    kf.Step(OCEAN_SERVER_IMU_INDEX, measurement_vector)

def encoders_imu_callback(data):
    # rospy.loginfo("Measure:\n%f\n%f", data.acceleration.x, data.acceleration.y)

    measurement_vector = numpy.matrix(
                            [ [data.linear.x],
                              [data.angular.z] ] )
    kf.Step(ENCODERS_INDEX, measurement_vector)

def gps_imu_callback(data):
    # rospy.loginfo("Measure:\n%f\n%f", data.acceleration.x, data.acceleration.y)

    measurement_vector = numpy.matrix(
                            [ [data.x],
                              [data.y] ] )
    kf.Step(GPS_INDEX, measurement_vector)


class ExtendedKalmanFilter:
    def __init__(self, _G_funct, _G_jacobian_funct, _H_functs, _H_jacobian_functs, _state, _P, _Q, _R_arr):
        self.G_funct = _G_funct                     # Transition function
        self.G_jacobian_funct = _G_jacobian_funct   # Transition jacobian
        self.H_functs = _H_functs                   # Observation functions
        self.H_jacobian_functs = _H_jacobian_functs # Observation matrices
        self.state = _state                         # Initial state estimate
        self.P = _P                                 # Initial covariance estimate
        self.Q = _Q                                 # Estimated variance in process
        self.R_arr = _R_arr                         # Estimated variances in measurements
        self.I = numpy.eye(_P.shape[0])             # Identity matrix
        self.prev_time = None

    def GetCurrentState(self):
        return self.state

    def CalcDT(self, cur_time):
        dt = 0
        if self.prev_time != None:
            dt = cur_time - self.prev_time
        self.prev_time = cur_time
        return dt

    def Predict(self):
        dt = self.CalcDT(rospy.get_time())

        G_jacobian = self.G_jacobian_funct(self.state, dt)

        #---------------------------Prediction step-----------------------------
        self.state = self.G_funct(self.state, dt)
        self.P = (G_jacobian * self.P) * numpy.transpose(G_jacobian) + self.Q

    def Step(self, sensor_index, measurement_vector):
        dt = self.CalcDT(time.time())

        G_jacobian = self.G_jacobian_funct(self.state, dt)

        #---------------------------Prediction step-----------------------------
        predicted_state = self.G_funct(self.state, dt)
        predicted_P = (G_jacobian * self.P) * numpy.transpose(G_jacobian) + self.Q

        #--------------------------Observation step-----------------------------
        H_funct = self.H_functs[sensor_index]
        H_jacobian = self.H_jacobian_functs[sensor_index](self.state, dt)
        R = self.R_arr[sensor_index]

        innovation = measurement_vector - H_funct(predicted_state, dt)
        innovation_covariance = H_jacobian*predicted_P*numpy.transpose(H_jacobian) + R

        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_P * numpy.transpose(H_jacobian) * numpy.linalg.inv(innovation_covariance)
        self.state = predicted_state + kalman_gain * innovation
        self.P = (self.I - kalman_gain*H_jacobian)*predicted_P


def imu_observation_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    a = state[5, 0]
    r = state[6, 0]
    p = state[7, 0]

    return numpy.matrix(\
        [ [a*math.cos(z + dt*w)],\
          [a*math.sin(z + dt*w)],\
          [r],\
          [p],\
          [z + dt*w] ])

def imu_jacobian_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
        [ [0, 0, -a*math.sin(z + dt*w), 0, -dt*a*math.sin(z + dt*w), math.cos(z + dt*w), 0, 0],\
          [0, 0,  a*math.cos(z + dt*w), 0,  dt*a*math.cos(z + dt*w), math.sin(z + dt*w), 0, 0],\
          [0, 0, 0, 0,  0, 0, 1, 0],\
          [0, 0, 0, 0,  0, 0, 0, 1],\
          [0, 0, 1, 0, dt, 0, 0, 0] ])

def encoders_observation_funct(state, dt):
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
        [ [v + dt*a],\
          [w] ])

def encoders_jacobian_funct(state, dt):
    return numpy.matrix(\
        [ [0, 0, 0, 1, 0, dt, 0, 0],\
          [0, 0, 0, 0, 1,  0, 0, 0] ])

def gps_observation_funct(state, dt):
    x = state[0, 0]
    y = state[1, 0]
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
    	  [ [x + (dt*v + 0.5*dt*dt*a)*math.cos(z + dt*w)],\
          [y + (dt*v + 0.5*dt*dt*a)*math.sin(z + dt*w)]])

def gps_jacobian_funct(state, dt):
    x = state[0, 0]
    y = state[1, 0]
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]
    r = state[6, 0]
    p = state[7, 0]

    return numpy.matrix(\
        [ [1, 0, -dt*(v + .5*a*dt)*math.sin(z + dt*w), dt*math.cos(z + dt*w),\
                 -dt*dt*(v + .5*a*dt)*math.sin(z + dt*w), .5*dt*dt*math.cos(z + dt*w), 0, 0],\
          [0, 1,  dt*(v + .5*a*dt)*math.cos(z + dt*w), dt*math.sin(z + dt*w),\
                  dt*dt*(v + .5*a*dt)*math.cos(z + dt*w), .5*dt*dt*math.sin(z + dt*w), 0, 0] ])


def transition_funct(state, dt):
    x = state[0, 0]
    y = state[1, 0]
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]
    r = state[6, 0]
    p = state[7, 0]

    arr = [ [x + (dt*v + 0.5*dt*dt*a)*math.cos(z + dt*w)],\
            [y + (dt*v + 0.5*dt*dt*a)*math.sin(z + dt*w)],\
            [z + dt*w],\
            [v + dt*a],\
            [w],\
            [a],\
            [r],\
            [p] ]

    return numpy.matrix(arr)

def transition_jacobian_funct(state, dt):
    z = state[2, 0]
    v = state[3, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
        [ [1, 0, -dt*(v + .5*a*dt)*math.sin(z + dt*w), dt*math.cos(z + dt*w),\
                 -dt*dt*(v + .5*a*dt)*math.sin(z + dt*w), .5*dt*dt*math.cos(z + dt*w), 0, 0],\
          [0, 1,  dt*(v + .5*a*dt)*math.cos(z + dt*w), dt*math.sin(z + dt*w),\
                  dt*dt*(v + .5*a*dt)*math.cos(z + dt*w), .5*dt*dt*math.sin(z + dt*w), 0, 0],\
          [0, 0, 1, 0, dt,  0, 0, 0],\
          [0, 0, 0, 1,  0, dt, 0, 0],\
          [0, 0, 0, 0,  1,  0, 0, 0],\
          [0, 0, 0, 0,  0,  1, 0, 0],\
          [0, 0, 0, 0,  0,  0, 1, 0],\
          [0, 0, 0, 0,  0,  0, 0, 1] ])


def create_EKF():
    initial_state = numpy.matrix([[0],[0],[0],[0],[0],[0],[0],[0]])
    initial_probability = numpy.eye(8)
    process_covariance = numpy.eye(8)*1e-3

    os_imu_measurement_covariance = numpy.eye(5)*0.0005
    encoders_measurement_covariance = numpy.eye(2)*1e-12
    gps_measurement_covariance = numpy.eye(2)*0.03

    return ExtendedKalmanFilter(transition_funct,\
                                transition_jacobian_funct,\
                                [imu_observation_funct, encoders_observation_funct, gps_observation_funct],\
                                [imu_jacobian_funct, encoders_jacobian_funct, gps_jacobian_funct],\
                                initial_state,\
                                initial_probability,\
                                process_covariance,\
                                [os_imu_measurement_covariance, encoders_measurement_covariance, gps_measurement_covariance])

def create_msg(belief):
    msg = EKFData()

    msg.x_pos = belief[0][0]
    msg.y_pos = belief[1][0]
    msg.linear_velocity = belief[3][0]
    msg.linear_acceleration = belief[5][0]

    msg.roll = belief[6][0]
    msg.pitch = belief[7][0]
    msg.yaw = belief[2][0]
    msg.yaw_rate = belief[4][0]

    return msg

if __name__ == '__main__':
    rospy.init_node('extended_kalman_filter', anonymous=True)

    kf = create_EKF()

    rospy.Subscriber("imu_rotated_data", RotatedIMUData, oceanserver_imu_callback)
    rospy.Subscriber("vel_data", Twist, encoders_imu_callback)
    rospy.Subscriber("gps_data", Point, gps_imu_callback)

    pub = rospy.Publisher('ekf_data', EKFData)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo("predicting & publishing!")

        kf.Predict()
        pub.publish(create_msg(kf.GetCurrentState()))

        r.sleep()



