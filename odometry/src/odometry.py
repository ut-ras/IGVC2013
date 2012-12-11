#!/usr/bin/env python
import roslib; roslib.load_manifest('odometry')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math
import tf

pub = rospy.Publisher('pose_raw',Pose)
br = tf.TransformBroadcaster()
t = rospy.Time
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
    p = Pose()
    dtime = float(rospy.Time.now().secs - t.secs)-(float(rospy.Time.now().nsecs - t.nsecs)/1000000000)
    angle += (data.angular.z - ((lastW - data.angular.z)/2)) * dtime
    angle = clamp(angle)
    quat = tf.transformations.quaternion_from_euler(0,0,angle)
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    v = (data.linear.x - ((lastV - data.linear.x)/2)) * dtime
    x += math.cos(angle) * v
    y += math.sin(angle) * v
    p.position.x = x
    p.position.y = y
    pub.publish(p)
    br.sendTransform((x,y,0),quat,rospy.Time.now(),"base_footprint","odom")
    t = rospy.Time.now()
    lastV = data.linear.x
    lastW = data.angular.z

def odometry():
    """Accumulation by trapizoidal Riemann sum
    of velocity data to determine odometry"""
    global t
    rospy.init_node('encoder_odometry')
    sub = rospy.Subscriber('vel_data',Twist,callback)
    t = rospy.Time.now()
    rospy.spin()

if __name__ == "__main__":
    try:
        odometry()
    except rospy.ROSInterruptException: pass

