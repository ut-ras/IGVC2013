#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ocean_server_imu.msg import custom_cool_msg

def talker():
    pub = rospy.Publisher('chatterzxcvxczvr', String)
    rospy.init_node('talkersdfgsdfg')

    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
