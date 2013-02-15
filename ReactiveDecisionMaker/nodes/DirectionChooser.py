import math

pi2 = math.pi*2
def angle_dif(a1, a2):
    a1 = (a1%pi2 + pi2)%pi2;
    a2 = (a2%pi2 + pi2)%pi2;
    d = abs(a1 - a2);
    
    if d > math.pi:
        d = pi2 - d;
    
    return d

class DirectionChooser:
    def __init__(self, MAXIMUM_CLEARANCE=1):
        self.CLEARANCE_WEIGHT = .3
        self.CURRENT_HEADING_WEIGHT = .1
        self.GOAL_HEADING_WEIGHT = .6
        self.MAXIMUM_CLEARANCE = MAXIMUM_CLEARANCE
    
    def pickBestDirection(self, directions, goalHeading, heading):
        bestDirIndex = -1
        bestScore = -1
        
        # loop through directions, pick best one
        for i in range(len(directions)):

            # normalize each value
            norm_clearance = directions[i].clearance/self.MAXIMUM_CLEARANCE
            norm_cur_heading = 1 - abs(angle_dif(directions[i].direction, heading))/math.pi
            norm_goal_heading = 1 - abs(angle_dif(directions[i].direction, goalHeading))/math.pi
            
            # multiple times weights & sum
            score = norm_clearance*self.CLEARANCE_WEIGHT\
                  + norm_cur_heading*self.CURRENT_HEADING_WEIGHT\
                  + norm_goal_heading*self.GOAL_HEADING_WEIGHT

            # compare with best score so far
            if score > bestScore:
                bestScore = score
                bestIndex = i

        return directions[bestIndex]
