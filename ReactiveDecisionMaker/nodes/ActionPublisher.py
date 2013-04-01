#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from ReactiveDecisionMaker.srv import GetAction
from geometry_msgs.msg import Twist

pub = None

if __name__ == "__main__":
    rospy.init_node('ActionPublisher')
    pub = rospy.Publisher('/vel_cmd', Twist)

    rospy.wait_for_service('getAction')
    getAction = rospy.ServiceProxy('getAction', GetAction)

    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            curAction = getAction().action
            pub.publish(curAction)        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        r.sleep()

    rospy.spin()
