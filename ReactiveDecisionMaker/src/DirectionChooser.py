import math
from ReactiveUtils import *
    
def pickBestDirection(directions, goalHeading, heading):
    bestDirIndex = -1
    bestScore = -1
    
    # loop through directions, pick best one
    for i in range(len(directions)):
        # normalize each value
        norm_clearance = directions[i].clearance/MAXIMUM_CLEARANCE
        norm_cur_heading = 1 - abs(angleDif(directions[i].direction, heading))/math.pi
        norm_goal_heading = 1 - abs(angleDif(directions[i].direction, goalHeading))/math.pi
        
        # print directions[i].direction, norm_clearance, norm_cur_heading, norm_goal_heading

        # multiple times weights & sum
        score = norm_clearance*CLEARANCE_WEIGHT\
              + norm_cur_heading*CURRENT_HEADING_WEIGHT\
              + norm_goal_heading*GOAL_HEADING_WEIGHT

        # compare with best score so far
        if score > bestScore:
            bestScore = score
            bestIndex = i

    return directions[bestIndex]
