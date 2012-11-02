#!/usr/bin/env python

import roslib; roslib.load_manifest('server_data_writer')
import rospy
import math

from sensor_msgs.msg import LaserScan
from ocean_server_imu.msg import RawData

def imu_callback(data):

    output = ""
    output += "roll:"+str(data.roll)+"\n"
    output += "pitch:"+str(data.pitch)+"\n"
    output += "yaw:"+str(data.yaw)+"\n"
    output += "offsetx:0\n"
    output += "offsety:0\n"
    output += "offsetz:0"

    o = open('/home/../var/www/imu.txt', 'w')
    o.write(output)
    o.close()

def imu_listener():
    rospy.init_node('imu_data_writer', anonymous=True)
    rospy.Subscriber("imu_data", RawData, imu_callback)

if __name__ == "__main__":
    imu_listener()
    rospy.spin()
