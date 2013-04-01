#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node('ActionPublisher')
    pub = rospy.Publisher('/vel_cmd', Twist)

    rospy.wait_for_service('getAction')
    getAction = rospy.ServiceProxy('getAction', GetAction)

    beingServiced = False
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        curAction = Twist()

        try:
            curAction = getAction().action
            pub.publish(curAction)

            if not beingServiced:
                print "Publishing data from GetAction service!"
                beingServiced = True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            beingServiced = False

        pub.publish(curAction)

        r.sleep()

    rospy.spin()
