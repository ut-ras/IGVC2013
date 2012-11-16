#!/usr/bin/env python
import roslib; roslib.load_manifest('odometry')
import rospy

import time
import math

from geometry_msgs.msg import Twist
from PSoC_Listener.msg import PSoC

# units in meters
WHEEL_AXIS_LENGTH = .5  
WHEEL_DIAMETER = 0.254
TICKS_PER_REVOLUTION = 8192.0

pub = None

heading = 0.0
x = 0.0
y = 0.0
prev_time = None
prev_left_enc = None
prev_right_enc = None

def enc_ticks_to_vel(encoder_ticks, delta_time):
    return (encoder_ticks*math.pi*WHEEL_DIAMETER/TICKS_PER_REVOLUTION)/delta_time

def update(x, y, heading, L, Vl, Vr, dt):
    new_x = None
    new_y = None 
    new_heading = None
            
    if abs(Vl - Vr) < .00001 :
        new_x = x + Vl*dt*math.cos(heading)
        new_y = y + Vl*dt*math.sin(heading)
        new_heading = heading
    else :
        # Credit: Dudek and Jenkin, Computational Principles of Mobile Robotics
        R = L*(Vl + Vr)/(2*(Vr - Vl))
        wd = dt*(Vr - Vl)/L
        new_x = x + R*math.cos(wd)*math.sin(heading) + R*math.sin(wd)*math.cos(heading) - R*math.sin(heading)
        new_y = y + R*math.sin(wd)*math.sin(heading) - R*math.cos(wd)*math.cos(heading) + R*math.cos(heading)
        new_heading = heading + wd
            
    new_heading = new_heading%(2*math.pi)

    return (new_x, new_y, new_heading)

def callback(data):
    global heading, x, y, prev_time, prev_left_enc, prev_right_enc

    if prev_time == None:
        prev_time = data.time
        prev_left_enc = data.left_enc
        prev_right_enc = data.right_enc
        return;

    cur_time = data.time
    dt = cur_time - prev_time
    prev_time = cur_time

    left_enc_delta = data.left_enc - prev_left_enc
    right_enc_delta = data.right_enc - prev_right_enc
   
    prev_left_enc = data.left_enc
    prev_right_enc = data.right_enc

    left_wheel_vel = enc_ticks_to_vel(left_enc_delta, dt)
    right_wheel_vel = enc_ticks_to_vel(right_enc_delta, dt)
    
    res = update(x, y, heading, WHEEL_AXIS_LENGTH, left_wheel_vel, right_wheel_vel, dt)

    x = res[0]
    y = res[1]
    heading = res[2]

    msg = Twist()
    msg.angular.z = heading
    msg.linear.x = x
    msg.linear.y = y
    pub.publish(msg)

    rospy.loginfo("Belief:\n"+str(msg));

if __name__ == "__main__":
    rospy.init_node('odometry_robz')
    rospy.Subscriber('psoc_data', PSoC, callback)
    pub = rospy.Publisher('pos_data_robz', Twist)
    rospy.spin()

