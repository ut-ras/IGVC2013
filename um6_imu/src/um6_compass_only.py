#!/usr/bin/env python
import roslib; roslib.load_manifest('um6_imu')
import rospy

import math

from um6_imu.msg import UM6IMU


def um6_imu_callback(data):
    yaw = math.atan2(data.compass.y, data.compass.x);
    rospy.loginfo("%.3f %.3f %.3f", data.compass.y, data.compass.x, yaw)

if __name__ == '__main__':
    rospy.init_node('um6_imu_processor', anonymous=False)

    rospy.Subscriber("um6_imu_data", UM6IMU, um6_imu_callback)

    rospy.spin()



