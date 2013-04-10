#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node('ZombiePublisher')
    pub = rospy.Publisher('/vel_cmd', Twist)

    r = rospy.Rate(10)

    action = Twist()
    action.angular.z = .1

    rospy.loginfo("publishing from zombie action!")

    while not rospy.is_shutdown():
        pub.publish(action)
        r.sleep()

    rospy.spin()
