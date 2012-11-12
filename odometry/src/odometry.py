#!/usr/bin/env python
import roslib; roslib.load_manifest('odometry')
import rospy
from geometry_msgs.msg import Twist
import time
import math

pub = rospy.Publisher('pos_data',Twist)
t = 0
angle = 0
x = 0
y = 0
lastV = 0
lastW = 0

def clamp(inp):
    if inp < -math.pi:
        return inp + (2*math.pi)
    elif inp > math.pi:
        return inp - (2*math.pi)
    else:
        return inp

def callback(data):
    global t
    global angle
    global x
    global y
    global lastV
    global lastW
    p = Twist()
    dtime = time.time() - t
    angle += (data.angular.z - ((lastW - data.angular.z)/2)) * dtime
    angle = clamp(angle)
    p.angular.z = angle
    v = (data.linear.x - ((lastV - data.linear.x)/2)) * dtime
    x += math.cos(angle) * v
    y += math.sin(angle) * v
    p.linear.x = x
    p.linear.y = y
    pub.publish(p)
    t = time.time()
    lastV = data.linear.x
    lastW = data.angular.z

def odometry():
    """Accumulation by trapizoidal Riemann sum
    of velocity data to determine odometry"""
    rospy.init_node('odometry')
    sub = rospy.Subscriber('vel_data',Twist,callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        t = time.time()
        odometry()
    except rospy.ROSInterruptException: pass

