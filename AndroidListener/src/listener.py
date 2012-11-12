#!/usr/bin/env python

import roslib; roslib.load_manifest('AndroidListener')
import rospy
from std_msgs.msg import String
import socket

host = 'localhost
socket = 32500
size = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host,port))
data = s.recv(size)
s.close()
print "Received:" , data


pub = rospy.Publisher('android_data',String)

def androidCall(data):
    print "Received:" , data

def listen():
    rospy.Subscriber("android_data", String, androidCall)
    rospy.spin()

if __name__ == "__main__":
    try:
        print "Starting ROS node"
        rospy.init_node('AndroidListener')
        listen()
    except rospy.ROSInterruptException: pass
