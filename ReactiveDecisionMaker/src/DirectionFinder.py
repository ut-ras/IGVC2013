#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math, pygame
from ReactiveUtils import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class Direction:
    def __init__(self, direction, clearance):
        self.direction = direction
        self.clearance = clearance
    def __repr__(self):
        return str(self.direction) + ", " + str(self.clearance)

endangles = None
enddists = None

def getEndangles():
    global endangles
    return endangles

def getEnddists():
    global enddists
    return enddists

def getViableDirections(shortenedLidar):
    global endangles, enddists
    endangles = []
    enddists = []
    viableDirections = []

    startGap = False
    numRanges = len(shortenedLidar)

    for i in range(numRanges):
        angle = shortenedLidar[i].angle
        dist = shortenedLidar[i].dist

        x = dist*math.cos(angle)
        y = dist*math.sin(angle)

        isMaxVal = abs(dist - MAX_VAL) < MAX_VAL_THREASHOLD

        if startGap and ((not isMaxVal) or i == numRanges - 1):
            startGap = False

            endangles.append(angle)
            enddists.append(dist)
        elif (not startGap) and isMaxVal and i != numRanges - 1:
            startGap = True

            endangles.append(angle)
            enddists.append(dist)

    for i in range(len(enddists)/2):
        index = i*2

        direction = averageAngles(endangles[index], endangles[index+1])
        if direction - math.pi < .0001:
            direction = 0
        print endangles[index], endangles[index+1], direction
        clearance = calcClearance(
            endangles[index], enddists[index],
            endangles[index+1], enddists[index+1]
        )

        if clearance > MIN_CLEARANCE_ALLOWED:
            viableDirections.append(Direction(direction, clearance))

    return viableDirections
