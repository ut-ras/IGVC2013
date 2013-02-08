#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    try:
        rospy.init_node('PSoC_Command')
        rospy.loginfo('PSoC_Command is running')
        print "Type messages to send raw commands to PSoC"
        pub = rospy.Publisher('psoc_cmd', String)
        while not rospy.is_shutdown():
            p = String()
            p.data = ">" + raw_input(">")
            pub.publish(p)
    except rospy.ROSInterruptException: pass
