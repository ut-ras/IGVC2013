#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, pylab, math, random, numpy, time

from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from ocean_server_imu.msg import RawData
from filters.msg import EKFData, Orientation
from um6_imu.msg import UM6IMU

from EKF import ExtendedKalmanFilter, accel_observation_funct, accel_jacobian_funct, rpy_observation_funct, rpy_jacobian_funct, encoders_observation_funct, encoders_jacobian_funct, gps_observation_funct, gps_jacobian_funct, transition_funct, transition_jacobian_funct


# OCEAN_SERVER_IMU_INDEX = 0
# UM6_IMU_INDEX = 0
# VN_200_IMU_INDEX = 0
ORIENTATION_INDEX = 0
ENCODERS_INDEX = 1
# UBLOX_GPS_INDEX = 2
VN_200_GPS_INDEX = 2


kf = None


def um6_imu_callback(data):
    measurement_vector = numpy.matrix(
            [ [data.orientation_euler.roll*math.pi/180.0],
              [data.orientation_euler.pitch*math.pi/180.0],
              [(-data.orientation_euler.yaw)*math.pi/180.0] ] )
    kf.Step(UM6_IMU_INDEX, measurement_vector)

def oceanserver_imu_callback(data):
    measurement_vector = numpy.matrix(
            [ [data.roll],
              [data.pitch],
              [data.yaw] ] )
    kf.Step(OCEAN_SERVER_IMU_INDEX, measurement_vector)

def vn_200_imu_callback(data):
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
def orientation_callback(data):
    curYaw = kf.GetCurrentState()[2,0]
    yaw = data.yaw

    yaw = yaw - 2*math.pi*math.floor((yaw - curYaw + math.pi)/(2*math.pi))

    measurement_vector = numpy.matrix(
                            [[data.roll],
                             [data.pitch],
                             [yaw]] )
    kf.Step(ORIENTATION_INDEX, measurement_vector)

"""
This corrects for the case where the state's yaw is
bounded from -inf to inf, whereas the update from
a sensor is bounded in some other way
"""
pi2 = math.pi*2
def compass_callback(roll, pitch, yaw, index):
    curYaw = kf.GetCurrentState()[2,0]

    yaw = yaw - 2*math.pi*math.floor((yaw - curYaw + math.pi)/(2*math.pi))

    measurement_vector = numpy.matrix(
                            [[roll],
                             [pitch],
                             [yaw]] )
    kf.Step(index, measurement_vector)

def encoders_callback(data):
    measurement_vector = numpy.matrix(
            [ [data.linear.x],
              [data.angular.z] ] )
    kf.Step(ENCODERS_INDEX, measurement_vector)

def gps_callback(data):
    measurement_vector = numpy.matrix(
                            [ [data.x],
                              [data.y] ] )
    kf.Step(GPS_INDEX, measurement_vector)

def create_EKF():
    initial_state = numpy.matrix([[0],[0],[0],[0],[0],[0],[0],[0]])
    initial_probability = numpy.eye(8)
    process_covariance = numpy.eye(8)*1e-3

    vn_200_imu_measurement_covar = numpy.eye(3)*1e-4
    encoders_measurement_covar = numpy.eye(2)*1e-6
    gps_measurement_covar = numpy.eye(2)*5.0

    return ExtendedKalmanFilter(
            transition_funct,
            transition_jacobian_funct,
            [rpy_observation_funct, encoders_observation_funct, gps_observation_funct],
            [rpy_jacobian_funct, encoders_jacobian_funct, gps_jacobian_funct],
            initial_state,
            initial_probability,
            process_covariance,
            [vn_200_imu_measurement_covar, encoders_measurement_covar, gps_measurement_covar])

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
    #rospy.Subscriber("orientation_data", Orientation, orientation_callback)
    rospy.Subscriber("vel_data", Twist, encoders_callback)
    #rospy.Subscriber("x_y_offseter", Point, gps_callback)

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
        rospy.Subscriber("vel_data", Twist, encoders_callback)
        topic_name += '_enc'

    if subscribe_list.find('yaw') != -1:
        rospy.Subscriber("orientation_data", Orientation, orientation_callback)
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

