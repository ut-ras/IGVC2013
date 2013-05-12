#!/usr/bin/env python
import rospy;
from std_msgs.msg import String

from Constants import *
from Geometry import GLib
from ClearanceCalculator import Circle
from Plotter import Plotter
from DynamicWindow import calcDynamicWindow

import roslib; roslib.load_manifest('DWDecisionMaker')

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
