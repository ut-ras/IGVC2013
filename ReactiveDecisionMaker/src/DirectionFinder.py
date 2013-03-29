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
        angle = shortendedLidar[i].angle
        dist = shortendedLidar[i].dist
        
        atMax = abs(MAX_VAL - dist) <= CLOSE_ENOUGH_TO_MAX
        
        nextAtMax = False
        if i < numRanges - 1:
            nextAtMax = abs(MAX_VAL - shortendedLidar[i + 1].dist)
                        <= CLOSE_ENOUGH_TO_MAX;
        
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

        angle1 = shortendedLidar[gapIndexes[i]].angle
        dist1 = shortendedLidar[gapIndexes[i]].dist
        angle2 = shortendedLidar[gapIndexes[i + 1]].angle
        dist2 = shortendedLidar[gapIndexes[i + 1]].dist
        
        angle = (angle1 + angle2)/2.0
        clearance = calcClearance(angle1, dist1, angle2, dist2)
        
        if clearance >= MIN_CLEARANCE_ALLOWED:    
            directions.append(Direction(angle, clearance))
            
            # add left-most direction
            for j in range(gapIndexes[i] + 1, gapIndexes[i + 1]):
                angleN = shortendedLidar[j].angle
                distN = shortendedLidar[j].dist
            
                angle = (angle1 + angleN)/2.0
                clearance = calcClearance(angle1, dist1, angleN, distN)
                
                if clearance > MIN_EDGE_CLEARANCE:
                    directions.append(Direction(angle, clearance))
                    break
                    
            # add right-most direction
            for j in range(gapIndexes[i + 1] - 1, gapIndexes[i], -1):
                angleN = shortendedLidar[j].angle
                distN = shortendedLidar[j].dist
            
                angle = (angle1 + angleN)/2.0
                clearance = calcClearance(angle1, dist1, angleN, distN)
                
                if clearance > MIN_EDGE_CLEARANCE:
                    directions.append(Direction(angle, clearance))
                    break
            
    return directions
"""
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

    print "endangles", endangles

    for i in range(len(enddists)/2):
        index = i*2

        direction = averageAngles(endangles[index], endangles[index+1])
        if abs(direction - math.pi) < .0001 or abs(direction + math.pi) < .0001:
            direction = 0

        clearance = calcClearance(
            endangles[index], enddists[index],
            endangles[index+1], enddists[index+1]
        )

        if clearance > MIN_CLEARANCE_ALLOWED:
            viableDirections.append(Direction(direction, clearance))

    return viableDirections
"""
