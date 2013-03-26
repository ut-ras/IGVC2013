import math

from ReactiveUtils import *
from geometry_msgs.msg import Twist

def getAction(direction, heading):
    action = Twist()

    # calculate angle difference, then normalize to [-1,1]
    error = angleDif(heading, direction)/(math.pi/2.0)

    # the smaller the error, the faster we move forward
    action.linear.x = MAX_V - MAX_V*abs(error)

    # the larger the err, the faster we turn
    action.angular.z = -error

    return action


