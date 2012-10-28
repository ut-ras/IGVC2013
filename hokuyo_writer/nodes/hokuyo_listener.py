#!/usr/bin/env python

import roslib; roslib.load_manifest('hokuyo_writer')
import rospy
from sensor_msgs.msg import LaserScan

import math

def callback(data):
    
    hokuyo_data = data.ranges
    current_angle = data.angle_min
    #rospy.loginfo(rospy.get_name() + "I hear data of type %s", type(angle))
    
    output = str(data.range_min) + " " + str(data.range_max) + "\n"
    
    z_val = 0.0
    
    for value in hokuyo_data:
        x_val = value * math.cos(current_angle + (math.pi/2))
        y_val = value * math.sin(current_angle + (math.pi/2))

        output += str(x_val) + " " + str(y_val) + " " + str(z_val) + "\n"
        current_angle += data.angle_increment

    o = open('../../../../var/www/hokuyo.txt', 'w')
    o.write(output)
    o.close()

def listener():
    rospy.init_node('hokuyo_writer', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)

    rospy.spin()

if __name__ == "__main__":
    listener()
