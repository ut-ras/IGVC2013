import math
from geometry_msgs.msg import Twist
from ReactiveUtils import ReactiveUtils

class DirectionFollower:
    def __init__(self, MAX_V=.5):
        self.MAX_V = MAX_V

    def getAction(self, direction, heading):
        action = Twist()

        # calculate angle difference, then normalize to [-1,1]
        error = ReactiveUtils.angle_dif(heading, direction)/(math.pi/2)

        # the smaller the error, the faster we move forward
        action.linear.x = self.MAX_V - self.MAX_V*abs(error)

        # the larger the err, the faster we turn
        action.angular.z = -error

        return action


