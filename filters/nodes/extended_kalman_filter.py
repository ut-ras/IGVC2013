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
from um6_imu.msg import UM6IMU
from vn_200_imu.msg import vn_200_ins_soln


USING_TEST1_BAGGED_DATA = False
USING_TEST2_BAGGED_DATA = False

#OCEAN_SERVER_IMU_INDEX = 0
#UM6_IMU_INDEX = 0
VN_200_IMU_INDEX = 0
ENCODERS_INDEX = 1
GPS_INDEX = 2


kf = None


def um6_imu_callback(data):
    if USING_TEST2_BAGGED_DATA:
        measurement_vector = numpy.matrix(
                                [ [data.orientation_euler.roll*math.pi/180.0],
                                  [data.orientation_euler.pitch*math.pi/180.0],
                                  [-data.orientation_euler.yaw*math.pi/180.0] ] )
    else:
        measurement_vector = numpy.matrix(
                                [ [data.orientation_euler.roll*math.pi/180.0],
                                  [data.orientation_euler.pitch*math.pi/180.0],
                                  [(-data.orientation_euler.yaw)*math.pi/180.0] ] )
    kf.Step(UM6_IMU_INDEX, measurement_vector)

def oceanserver_imu_callback(data):
    measurement_vector = numpy.matrix(
                            [ #[data.acceleration.x],
                              #[data.acceleration.y],
                              [data.roll],
                              [data.pitch],
                              [data.yaw] ] )
    kf.Step(OCEAN_SERVER_IMU_INDEX, measurement_vector)

def vn_200_imu_callback(data):
    """
    measurement_vector = numpy.matrix(
                            [ [data.orientation_euler.roll*math.pi/180.0],
                              [data.orientation_euler.pitch*math.pi/180.0],
                              [-(data.orientation_euler.yaw)*math.pi/180.0] ] )
    kf.Step(VN_200_IMU_INDEX, measurement_vector)
    """
    compass_callback(
            data.orientation_euler.roll*math.pi/180.0,
            data.orientation_euler.pitch*math.pi/180.0,
            -(data.orientation_euler.yaw)*math.pi/180.0,
            VN_200_IMU_INDEX
            )

"""
This corrects for the case where the state's yaw is
bounded from -inf to inf, whereas the update from
a sensor is bounded in some other way
"""
pi2 = math.pi*2
def compass_callback(roll, pitch, yaw, index):
    curYaw = kf.GetCurrentState()[2,0]

    bounded_curYaw = (curYaw%pi2 + pi2)%pi2
    bounded_yaw = (yaw%pi2 + pi2)%pi2

    yaw_r = bounded_yaw - bounded_curYaw
    yaw_f = curYaw + yaw_r

    measurement_vector = numpy.matrix(
                            [[roll],
                             [pitch],
                             [yaw_f]] )
    kf.Step(index, measurement_vector)

def encoders_imu_callback(data):
    if USING_TEST1_BAGGED_DATA:
        measurement_vector = numpy.matrix(
                            [ [data.linear.x],
                              [-data.angular.z] ] )
    else:
        measurement_vector = numpy.matrix(
                            [ [data.linear.x],
                              [data.angular.z] ] )
    kf.Step(ENCODERS_INDEX, measurement_vector)

def gps_callback(data):
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

    def GetCurrentCovMatrix(self):
        return self.P

    def GetCurrentState(self):
        return self.state

    def CalcDT(self, cur_time):
        dt = 0
        if self.prev_time != None:
            dt = cur_time - self.prev_time
        self.prev_time = cur_time
        return dt

    def Predict(self, dt=None):
        if dt == None:
            dt = self.CalcDT(rospy.get_time())

        G_jacobian = self.G_jacobian_funct(self.state, dt)

        #---------------------------Prediction step-----------------------------
        self.state = self.G_funct(self.state, dt)
        self.P = (G_jacobian * self.P) * numpy.transpose(G_jacobian) + self.Q

    def Step(self, sensor_index, measurement_vector, dt=None):
        if dt == None:
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

def accel_observation_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
        [ [a*math.cos(z + dt*w)],\
          [a*math.sin(z + dt*w)] ])


def accel_jacobian_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    a = state[5, 0]

    return numpy.matrix(\
        [ [0, 0, -a*math.sin(z + dt*w), 0, -dt*a*math.sin(z + dt*w), math.cos(z + dt*w), 0, 0],\
          [0, 0,  a*math.cos(z + dt*w), 0,  dt*a*math.cos(z + dt*w), math.sin(z + dt*w), 0, 0] ])

def compass_observation_funct(state, dt):
    z = state[2, 0]
    w = state[4, 0]
    r = state[6, 0]
    p = state[7, 0]

    return numpy.matrix(\
        [ [r],\
          [p],\
          [z + dt*w] ])

def compass_jacobian_funct(state, dt):
    return numpy.matrix(\
        [ [0, 0, 0, 0,  0, 0, 1, 0],\
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

    # os_imu_measurement_covariance = numpy.eye(3)*1e-6
    # um6_imu_measurement_covariance = numpy.eye(3)*1e-6
    vn_200_imu_measurement_covariance = numpy.eye(3)*.17
    encoders_measurement_covariance = numpy.eye(2)*1e-6
    gps_measurement_covariance = numpy.eye(2)*5.0

    return ExtendedKalmanFilter(transition_funct,\
                                transition_jacobian_funct,\
                                [compass_observation_funct, encoders_observation_funct, gps_observation_funct],\
                                [compass_jacobian_funct, encoders_jacobian_funct, gps_jacobian_funct],\
                                initial_state,\
                                initial_probability,\
                                process_covariance,\
                                #[os_imu_measurement_covariance, encoders_measurement_covariance, gps_measurement_covariance])
                                #[um6_imu_measurement_covariance, encoders_measurement_covariance, gps_measurement_covariance])
                                [vn_200_imu_measurement_covariance, encoders_measurement_covariance, gps_measurement_covariance])

def create_msg(belief, covariances):
    msg = EKFData()

    msg.header.stamp = rospy.get_rostime()

    msg.x_pos = belief[0,0]
    msg.y_pos = belief[1,0]
    msg.linear_velocity = belief[3,0]
    msg.linear_acceleration = belief[5,0]

    msg.roll = belief[6,0]
    msg.pitch = belief[7,0]
    msg.yaw = belief[2,0]
    msg.yaw_rate = belief[4,0]

    msg.variances.x_pos = covariances[0,0]
    msg.variances.y_pos = covariances[1,1]
    msg.variances.linear_velocity = covariances[3,3]
    msg.variances.linear_acceleration = covariances[5,5]

    msg.variances.roll = covariances[6,6]
    msg.variances.pitch = covariances[7,7]
    msg.variances.yaw = covariances[2,2]
    msg.variances.yaw_rate = covariances[4,4]

    return msg

def init_old():
    #rospy.Subscriber("um6_imu_data", UM6IMU, um6_imu_callback)
    #rospy.Subscriber("imu_rotated_data", RotatedIMUData, oceanserver_imu_callback)
    rospy.Subscriber("vn_200_ins_soln", vn_200_ins_soln, vn_200_imu_callback)
    rospy.Subscriber("vel_data", Twist, encoders_imu_callback)
    rospy.Subscriber("x_y_offseter", Point, gps_callback)

    pub = rospy.Publisher('ekf_data', EKFData)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo("predicting & publishing!")

        kf.Predict()
        pub.publish(create_msg(kf.GetCurrentState(), kf.GetCurrentCovMatrix()))

        r.sleep()

def init_new():
    subscribe_list = str(rospy.get_param('~topics'))
    topic_name = 'ekf_data'

    if subscribe_list.find('enc') != -1:
        rospy.Subscriber("vel_data", Twist, encoders_imu_callback)
        topic_name += '_enc'

    if subscribe_list.find('yaw') != -1:
        rospy.Subscriber("vn_200_ins_soln", vn_200_ins_soln, vn_200_imu_callback)
        topic_name += '_yaw'

    if subscribe_list.find('gps') != -1:
        rospy.Subscriber("x_y_offseter", Point, gps_callback)
        topic_name += '_gps'

    pub = rospy.Publisher(topic_name, EKFData)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo("predicting & publishing!")

        kf.Predict()
        pub.publish(create_msg(kf.GetCurrentState(), kf.GetCurrentCovMatrix()))

        r.sleep()

def init():
    rospy.init_node('extended_kalman_filter', anonymous=False)

    global kf
    kf = create_EKF()

    subscribe_from_params = False
    try:
        subscribe_from_params = bool(rospy.get_param('~custom_subscribers'))
    except KeyError: pass

    if subscribe_from_params:
        init_new()
    else:
        init_old()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

