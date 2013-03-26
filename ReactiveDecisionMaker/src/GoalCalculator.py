import math, pygame

from DirectionFinder import Direction
from ReactiveUtils import *

def calcGoalHeading(curPos, goalPos):
    dx = goalPos.x - curPos.x
    dy = goalPos.y - curPos.y
    return math.atan2(dy, dx)

def calcViableDir(goalHeading, shortenedLidar, heading, curPos, goalPos):
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
        if abs(shortenedLidar[i].dist - MAX_VAL) > MAX_VAL_THREASHOLD:
            break
        leftIndex = i

    ### go right until hitting a bump
    rightIndex = index
    for i in range(index, -1, -1):
        if abs(shortenedLidar[i].dist - MAX_VAL) > MAX_VAL_THREASHOLD:
            break
        rightIndex = i

    ### if there was enough clearance in both directions, return the direction
    clearance1 = calcClearance(
        shortenedLidar[leftIndex].angle, shortenedLidar[leftIndex].dist,
        shortenedLidar[index].angle, shortenedLidar[index].dist)

    clearance2 = calcClearance(
        shortenedLidar[rightIndex].angle, shortenedLidar[rightIndex].dist,
        shortenedLidar[index].angle, shortenedLidar[index].dist)

    if clearance1 > MIN_CLEARANCE_ALLOWED/2.0 and \
       clearance2 > MIN_CLEARANCE_ALLOWED/2.0:
        return Direction(goalHeading, 2.0*min(clearance1, clearance2))

    ### if there wasn't enough clearance, see if the goal is close enough
    ### to not have to worry about clearance
    goalDistance = euclidDistPoint(goalPos, curPos)

    if min(shortenedLidar[leftIndex].dist, 
           shortenedLidar[rightIndex].dist) > goalDistance:
        return Direction(goalHeading, MIN_CLEARANCE_ALLOWED)

    return None


