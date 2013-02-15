import math, pygame

from DirectionFinder import Direction

MAX_VAL_THREASHOLD = 1e-3 # precision around max

pi2 = math.pi*2.0
def boundAngleToNPItoPI(angle):
    angle = (angle%pi2 + pi2)%pi2;
    if angle > math.pi:
        angle -= pi2
    return angle

def euclidDist2D(p0, p1):
    return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

def euclidDist2DT(p0, p1):
    return math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

def calcClearance(curPos, angle1, dist1, angle2, dist2) :
    dist = min(dist1, dist2)

    p0 = (curPos.x + dist*math.cos(angle1),\
	      curPos.y + dist*math.sin(angle1))
    p1 = (curPos.x + dist*math.cos(angle2),\
	      curPos.y + dist*math.sin(angle2))

    return euclidDist2DT(p0, p1)


class GoalCalculator:
    def __init__(self, SHOW_GRAPHICS=True, MAX_VAL=1, MIN_CLEARANCE_ALLOWED=.61):
        self.SHOW_GRAPHICS = SHOW_GRAPHICS
        self.MAX_VAL = MAX_VAL
        self.MIN_CLEARANCE_ALLOWED = MIN_CLEARANCE_ALLOWED

    def calcGoalHeading(self, curPos, goalPos):
        dx = goalPos.x - curPos.x
        dy = goalPos.y - curPos.y
        return math.atan2(dy, dx)

    def calcViableDir(self, goalHeading, shortenedLidar, heading, curPos, goalPos):
        goalHeadingTemp = boundAngleToNPItoPI(goalHeading - heading)
        
        # check to see if the goal direction is within lidar view
        if goalHeadingTemp > math.pi/2.0 or goalHeadingTemp < -math.pi/2.0:
            return None

        # check clearance aroud lidar to see if it fits
        ### figure out which lidarValue index goalHeading best fits 
        numVals = len(shortenedLidar)
        index = int((goalHeadingTemp + math.pi/2.0)/(math.pi)*numVals)

        if index >= numVals:
            index = numVals - 1
        elif index < 0:
            print "index, you're too fucking small! ", index
            index = 0
            

        ### go left until hitting a bump
        leftIndex = index
        for i in range(index, numVals):
            if abs(shortenedLidar[i].dist - self.MAX_VAL) > MAX_VAL_THREASHOLD:
                break
            leftIndex = i
        
        ### go right until hitting a bump
        rightIndex = index
        for i in range(index, -1, -1):
            if abs(shortenedLidar[i].dist - self.MAX_VAL) > MAX_VAL_THREASHOLD:
                break
            rightIndex = i

        ### if there was enough clearance in both directions, return the direction
        clearance1 = calcClearance(curPos,\
            shortenedLidar[leftIndex].angle, shortenedLidar[leftIndex].dist,\
            shortenedLidar[index].angle, shortenedLidar[index].dist)

        clearance2 = calcClearance(curPos,\
            shortenedLidar[rightIndex].angle, shortenedLidar[rightIndex].dist,\
            shortenedLidar[index].angle, shortenedLidar[index].dist)
        
        if clearance1 > self.MIN_CLEARANCE_ALLOWED/2.0 and \
           clearance2 > self.MIN_CLEARANCE_ALLOWED/2.0:
            return Direction(goalHeading, clearance1 + clearance2)

        ### if there wasn't enough clearance, see if the goal is close enough
        ### to have to worry about clearance
        goalDistance = euclidDist2D(goalPos, curPos)

        if min(shortenedLidar[leftIndex].dist, shortenedLidar[rightIndex].dist) > goalDistance:
            return Direction(goalHeading, self.MIN_CLEARANCE_ALLOWED)

        return None
















