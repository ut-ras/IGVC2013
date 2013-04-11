import math

from ReactiveUtils import *
from geometry_msgs.msg import Twist

def getAction(direction, heading, angleRange):
    action = Twist()

    # calculate angle difference, then normalize to [-1,1]
    error = angleDif(heading, direction)/(angleRange/2.0)

    # the smaller the error, the faster we move forward
    action.linear.x = MAX_V - MAX_V*abs(error)

    # the larger the err, the faster we turn
    action.angular.z = -error*MAX_ANGULAR

    return action


