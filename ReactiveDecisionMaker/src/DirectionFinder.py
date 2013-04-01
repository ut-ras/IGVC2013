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
        return "(" + str(self.direction) + ", " + str(self.clearance) + ")"

endangles = None
enddists = None

def getEndangles():
    global endangles
    return endangles

def getEnddists():
    global enddists
    return enddists

def calcViableDirs(shortenedLidar):
    numRanges = len(shortenedLidar)
    inGap = False
    gapIndexes = []

    for i in range(numRanges):
        angle = shortenedLidar[i].angle
        dist = shortenedLidar[i].dist

        atMax = abs(MAX_VAL - dist) <= MAX_VAL_PRECISION

        nextAtMax = False
        if i < numRanges - 1:
            nextAtMax = abs(MAX_VAL - shortenedLidar[i + 1].dist) \
                        <= MAX_VAL_PRECISION;

        if i == 0 and atMax:
            gapIndexes.append(i)
            inGap = True
        elif inGap and not atMax:
            gapIndexes.append(i)
            inGap = False
        elif not inGap and nextAtMax:
            gapIndexes.append(i)
            inGap = True
        elif inGap and i == numRanges - 1:
            gapIndexes.append(i)
            inGap = False

    directions = []

    for i in range(len(gapIndexes)/2):
        i *= 2

        angle1 = shortenedLidar[gapIndexes[i]].angle
        dist1 = shortenedLidar[gapIndexes[i]].dist
        angle2 = shortenedLidar[gapIndexes[i + 1]].angle
        dist2 = shortenedLidar[gapIndexes[i + 1]].dist

        angle = (angle1 + angle2)/2.0
        clearance = calcClearance(angle1, dist1, angle2, dist2)

        if clearance >= MIN_CLEARANCE_ALLOWED:
            directions.append(Direction(angle, clearance))

            # add left-most direction
            for j in range(gapIndexes[i] + 1, gapIndexes[i + 1]):
                angleN = shortenedLidar[j].angle
                distN = shortenedLidar[j].dist

                angle = (angle1 + angleN)/2.0
                clearance = calcClearance(angle1, dist1, angleN, distN)

                if clearance > MIN_EDGE_CLEARANCE:
                    directions.append(Direction(angle, clearance))
                    break

            # add right-most direction
            for j in range(gapIndexes[i + 1] - 1, gapIndexes[i], -1):
                angleN = shortenedLidar[j].angle
                distN = shortenedLidar[j].dist

                angle = (angle2 + angleN)/2.0
                clearance = calcClearance(angle2, dist2, angleN, distN)

                if clearance > MIN_EDGE_CLEARANCE:
                    directions.append(Direction(angle, clearance))
                    break

    return directions
