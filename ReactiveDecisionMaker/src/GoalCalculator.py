import math, pygame

from DirectionFinder import Direction
from ReactiveUtils import *

def calcGoalHeading(curPos, goalPos):
    dx = goalPos.x - curPos.x
    dy = goalPos.y - curPos.y
    return math.atan2(dy, dx)

def calcViableDir(
        goalHeading,
        shortenedLidar,
        heading,
        curPos,
        goalPos,
        startAngle,
        angleRange
        ):

    goalHeadingTemp = boundAngleToNPItoPI(goalHeading - heading)

    # check to make sure the goal direction is within lidar view
    if goalHeadingTemp > startAngle + angleRange or goalHeadingTemp < startAngle:
        return None

    # check clearance aroud goal direction to see if it's large enough
    ### first, figure out which lidarValue index goalHeading best fits
    numVals = len(shortenedLidar)
    index = int((goalHeadingTemp - startAngle)/(angleRange)*numVals)

    if index >= numVals:
        index = numVals - 1
    elif index < 0:
        print "index, you're too fucking small! ", index
        index = 0

    ### go left until hitting a bump
    leftIndex = index
    for i in range(index, numVals):
        leftIndex = i
        if abs(shortenedLidar[i].dist - MAX_VAL) > MAX_VAL_PRECISION:
            break

    ### go right until hitting a bump
    rightIndex = index
    for i in range(index, -1, -1):
        rightIndex = i
        if abs(shortenedLidar[i].dist - MAX_VAL) > MAX_VAL_PRECISION:
            break

    ### calulate the clearances in both directions
    clearance1 = calcClearance(
        shortenedLidar[leftIndex].angle, shortenedLidar[leftIndex].dist,
        shortenedLidar[index].angle, shortenedLidar[index].dist)

    clearance2 = calcClearance(
        shortenedLidar[rightIndex].angle, shortenedLidar[rightIndex].dist,
        shortenedLidar[index].angle, shortenedLidar[index].dist)

    ### if there's enough clearance on both sides, return double the lesser one
    if clearance1 > MIN_CLEARANCE_ALLOWED/2.0 and \
       clearance2 > MIN_CLEARANCE_ALLOWED/2.0:
        return Direction(goalHeading, 2.0*min(clearance1, clearance2))

    ### if there wasn't enough clearance, see if the goal is close enough
    ### to not have to worry about clearance
    goalDistance = euclidDistPoint(goalPos, curPos)

    if min(shortenedLidar[leftIndex].dist,
           shortenedLidar[rightIndex].dist) > goalDistance:
        return Direction(goalHeading, MIN_CLEARANCE_ALLOWED) # may be unwise to have MIN, not sure

    return None


