#!/usr/bin/env python

import roslib; roslib.load_manifest('server_data_writer')
import rospy
import math

from sensor_msgs.msg import LaserScan

def hokuyo_callback(data):

    hokuyo_data = data.ranges
    current_angle = data.angle_min
    
    output = str(data.range_min) + " " + str(data.range_max) + "\n"
    
    z_val = 0.0
    
    for value in hokuyo_data:
        x_val = value * math.cos(current_angle + (math.pi/2))
        y_val = value * math.sin(current_angle + (math.pi/2))

        output += str(x_val) + " " + str(y_val) + " " + str(z_val) + "\n"
        current_angle += data.angle_increment

    o = open('/home/../var/www/hokuyo.txt', 'w')
    o.write(output)
    o.close()

def hokuyo_listener():
    rospy.init_node('hokuyo_writer', anonymous=True)
    rospy.Subscriber('scan', LaserScan, hokuyo_callback)

if __name__ == "__main__":
    hokuyo_listener()
    rospy.spin()
