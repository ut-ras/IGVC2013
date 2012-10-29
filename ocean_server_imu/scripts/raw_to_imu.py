#!/usr/bin/python
PKG = "ocean_server_imu"    # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
from ocean_server_imu.msg import RawData
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler
import math

def from_degrees(deg):
    return deg * math.pi / 180.0

def callback(raw_data, pub):
    imu = Imu()
    imu.header = raw_data.header

    # set imu.orientation quaternion
    # Btw, robot_pose_ekf expects the Roll and Pitch angles to be
    # absolute, but the Yaw angle to be relative. Compute the relative
    # angle from the currently reported and previously reported absolute
    # angle
    r = raw_data.roll
    p = raw_data.pitch
    y = raw_data.yaw - callback.old_yaw
    callback.old_yaw = raw_data.yaw

    quat = quaternion_from_euler(r,p,y,"sxyz")

    imu.orientation.x = quat[0]
    imu.orientation.y = quat[1]
    imu.orientation.z = quat[2]
    imu.orientation.w = quat[3]

    #TODO: set imu.orientation_covariance
    #row major about x, y, z
    # According to OS5000 datasheet, accuracy
    # depends on tilt:
    # 0.5 deg RMS level heading,
    # 1 deg typical RMS accuracy < +-30deg tilt,
    # 1.5deg < +-60deg tilt
    # Roll and Pitch:
    # Typical 1deg accuracy for tilt < +-30 deg
    # Assume tilt < +- 30 deg, set all standard
    # deviations to 1 degree
    std_dev = from_degrees(1)
    imu.orientation_covariance[0] = std_dev**2
    imu.orientation_covariance[4] = std_dev**2
    # standard deviation on yaw is doubled since
    # it's computed as a difference of two measurements,
    # each with a std_dev of 1 degree
    imu.orientation_covariance[8] = (2*std_dev)**2

    # no angular velocity data is available, so set
    # element 0 of covariance matrix to -1
    # alternatively, set the covariance really high
    imu.angular_velocity = Vector3(0,0,0)
    imu.angular_velocity_covariance[0] = -1

    # Set linear acceleration
    imu.linear_acceleration = raw_data.acceleration
    # TODO: Set linear acceleration covariance
    imu.linear_acceleration_covariance[0] = -1

    pub.publish(imu)

# initialize static-ish variable in callback() function
callback.old_yaw = 0

def listener():
    rospy.init_node("raw_to_imu")

    pub = rospy.Publisher("~imu", Imu)
    rospy.Subscriber("imu_data", RawData, callback, pub)

    rospy.spin()

if __name__ == '__main__':
    listener()
