#!/usr/bin/env python
import roslib; roslib.load_manifest('navCalib')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from PSoC_msgs.msg import PSoC
import time

#Test driving calibration for encoders, sensors, etc...
#Estimated 8192ticks/rotation of a wheel

pub = rospy.Publisher('psoc_cmd',String)

# p.left_enc p.right_enc p.vel_v p.vel_w p.time

data = Null

def psocCall(dat):
    data = dat

def listen():
    rospy.Subscriber("psoc_data", PSoC, psocCall)
    rospy.spin()

def clamp(a,x,y):
    if (a<x):
        return x
    if (a>y):
        return y
    return a

def drive(power,turn):
    turn = clamp(turn,0,255)
    msg = String()
    msg.data = ">SVXA:"
    msg.data += turn
    rospy.loginfo("Turning with V="+turn)
    pub.publish(msg)

    power = clamp(power,0,255)
    msg = String()
    msg.data = ">SVYA:"
    msg.data += power
    rospy.loginfo("Driving with V="+power)
    pub.publish(msg)

    time.sleep(.01)

def getLeftEnc():
    return data.left_enc

def getRightEnc():
    return data.right_end

def getVelocity():
    return data.vel_v

def getAngularVelocity():
    return data.vel_w

def getTime():
    return data.time

def resetEnc():
    msg = String()
    msg.data = ">RSTE"
    rospy.loginfo("Reset Encoder Data!")
    pub.publish(msg)
    
if __name__ == "__main__":
    try:
        print "Starting ROS node"
        rospy.init_node('navCalib')
        listen()
        time.sleep(.1)
        while (getLeftEnc() < 8192):
            drive(100,0)
    except rospy.ROSInterruptException: pass
