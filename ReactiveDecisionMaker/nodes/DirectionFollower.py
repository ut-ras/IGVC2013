import math
from geometry_msgs.msg import Twist

pi2 = math.pi*2
def angle_dif(a1, a2):
    a1 = (a1%pi2 + pi2)%pi2;
    a2 = (a2%pi2 + pi2)%pi2;
    d = abs(a1 - a2);
    
    if d > math.pi:
        d = pi2 - d;
    
    return d

def getCrossCheckError(heading, direction):
    return angle_dif(heading, direction)/math.pi

class DirectionFollower:
    def __init__(self, MAX_V=1):
        self.error_sum = 0
        self.old_error = None
        self.MAX_V = MAX_V
        self.P_TERM = 10
        self.D_TERM = 2
        self.I_TERM = 0

    def getAction(self, direction, heading):
        action = Twist()

        error = getCrossCheckError(heading, direction)
        
        if self.old_error == None:
            self.old_error = error

        delta_steering_angle = -self.P_TERM*error \
                             + -self.D_TERM*(error - self.old_error) \
                             + -self.I_TERM*self.error_sum

        self.old_error = error
        self.error_sum += error

        left_wheel_vel = None
        right_wheel_vel = None

        if delta_steering_angle > 0:
		    left_wheel_vel = self.MAX_V - self.MAX_V*delta_steering_angle/math.pi
		    right_wheel_vel = self.MAX_V

		    if left_wheel_vel > self.MAX_V:
			    left_wheel_vel = self.MAX_V
		    elif left_wheel_vel < -self.MAX_V:
			    left_wheel_vel = -self.MAX_V
        elif delta_steering_angle < 0:
		    left_wheel_vel = self.MAX_V
		    right_wheel_vel = self.MAX_V + self.MAX_V*delta_steering_angle/math.pi
		
		    if right_wheel_vel > self.MAX_V:
			    right_wheel_vel = self.MAX_V
		    elif right_wheel_vel < -self.MAX_V:
			    right_wheel_vel = -self.MAX_V
        else:
		    left_wheel_vel = self.MAX_V;
		    new_wheel2_vel = self.MAX_V;	

        action.linear.x = (left_wheel_vel + right_wheel_vel)/2
        action.angular.z = (left_wheel_vel - right_wheel_vel)/2

        return action


