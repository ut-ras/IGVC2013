#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub_data = rospy.Publisher('vel_data', Twist)
pub_cmd = rospy.Publisher('psoc_cmd', String)
rate = 20;
def dataCallback(data):
    p = Twist()
    rate = data.rate;
    p.linear.x = float(data.vel_v) * .00004871 * 1000 / rate
    p.angular.z = float(data.vel_w) * .00019482 * 1000 / rate
    pub_data.publish(p)

def cmdCallback(data):
    p = String()
    p.data = ">SVLX:"+str(data.linear.x / .00004871 / 1000 / rate)
    pub_cmd.publish(p)
    p.data = ">SVAZ:"+str(data.angular.z / .00019482 / 1000 / rate)
    pub_cmd.publish(p)

def velocities():
    """Conversions between encoder outputs to meters per second"""
    rospy.init_node('velocity_conversion')
    sub = rospy.Subscriber('psoc_data', PSoC, dataCallback)
    sub = rospy.Subscriber('vel_cmd', Twist, cmdCallback)
    rospy.spin()

if __name__ == "__main__":
    try:
        velocities()
    except rospy.ROSInterruptException: pass
