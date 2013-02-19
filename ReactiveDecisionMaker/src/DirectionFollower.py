import math
from geometry_msgs.msg import Twist
from ReactiveUtils import ReactiveUtils

def getCrossCheckError(heading, direction):
    return ReactiveUtils.angle_dif(heading, direction)/(math.pi/2)

class DirectionFollower:
    def __init__(self, MAX_V=.5):
        self.error_sum = 0
        self.old_error = None
        self.MAX_V = MAX_V
        self.P_TERM = 10
        self.D_TERM = 2
        self.I_TERM = 0

    def getAction(self, direction, heading):
        action = Twist()

        error = getCrossCheckError(heading, direction)

        #action.linear.x = self.MAX_V - self.MAX_V*abs(error)
        action.angular.z = error/16.0

        print heading, direction, error, action.linear.x, action.angular.z

        return action


