#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from PSoC_Listener.srv import GetLatestAngle
from std_msgs.msg import Float64
from math import pi as PI

pub = rospy.Publisher('hokuyo_angle', Float64)
lastValue = float(0)
MAX_ANGLE = PI / 18
MAX_INPUT = float(-400)
ANGLE_OFFSET = float(0)

def service(req):
    return lastValue

def callback(data):
    global lastValue
    p = Float64()
    p.data = (float(data.adc) * MAX_ANGLE / MAX_INPUT) + ANGLE_OFFSET
    lastValue = p.data
    pub.publish(p)

def angle():
    rospy.init_node('PSoC_ADC_To_Angle')
    sub = rospy.Subscriber('psoc_data', PSoC, callback)
    srv = rospy.Service('latest_hokuyo_angle', GetLatestAngle, service)
    rospy.loginfo('PSoC_ADC_To_Angle is running.')
    rospy.spin()

if __name__ == "__main__":
    try:
        angle()
    except rospy.ROSInterruptetException: pass

