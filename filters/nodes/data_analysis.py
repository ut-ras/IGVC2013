#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point
from filters.msg import RotatedIMUData,EKFData

x_count = 0
x_sum = 0
x_squared_sum = 0

def oceanserver_imu_callback(data):
    rospy.loginfo("received imu data!");
    ax = data.acceleration.x

    global x_count, x_sum, x_squared_sum
    x_count += 1.0
    x_sum += ax
    x_squared_sum += ax**2

    x_mean = x_sum/x_count
    x_squared_mean = x_squared_sum/x_count
    x_var = x_squared_mean - x_mean**2

    rospy.loginfo("mean: %f\nvar: %f", x_mean, x_var);

if __name__ == '__main__':
    rospy.init_node('data_analysis', anonymous=True)

    rospy.Subscriber("imu_rotated_data", RotatedIMUData, oceanserver_imu_callback)

    rospy.spin()
