#! /usr/bin/env python

import math

class ScanLine:
    def __init__(self, oindex, odist, oangle, isMax):
        self.oindex = oindex
        self.odist = odist
        self.oangle = oangle
        self.isMax = isMax
        
    def setLeftVals(self, lindex, langle):
        self.lindex = lindex
        self.langle = langle
    
    def setRightVals(self, rindex, rangle):
        self.rindex = rindex
        self.rangle = rangle

class Gap:
    def __init__(self, index, angle, clearance):
        self.index = index
        self.angle = angle 
        self.clearance = clearance

# expecting scan to be an array of ScanLines and gaps to be an arry of Gaps
def calcClearances(scan, gaps, width):
    scanLen = len(scan)

    # calculate scan values in the left and right corner's reference frame
    for line in scan:
        px = line.odist*math.cos(line.oangle)
        
        #left
        py = line.odist*math.sin(line.oangle) + width/2.0
        theta = math.atan2(py, px)
        line.setLeftVals(-1, theta)
        
        # right
        py = line.odist*math.sin(line.oangle) - width/2.0
        theta = math.atan2(py, px)
        line.setRightVals(-1, theta)

    # create two new lists, each sorted by the angle of i
    #  left and right corner scans respectively 
    leftList = sorted(scan, key=lambda line: line.langle)
    rightList = sorted(scan, key=lambda line: line.rangle)

    # set the left and right indexes
    # remember: 'sorted' creates a shallow copy of lists
    for i in range(scanLen):
        leftList[i].lindex = i
    
    for i in range(scanLen):
        rightList[i].rindex = i

    # now loop through the gaps
    for gap in gaps:
        # this is the magic part
        startl = scan[gap.index].lindex
        startr = scan[gap.index].rindex

        # see how far we can go left
        endl = startl
        while endl < scanLen-1:
            if not leftList[endl].isMax:
                break
            endl += 1
        
        # & how far can we go right 
        endr = startr
        while endr > 0:
            if not rightList[endr].isMax:
                break
            endr -= 1

        # calc clearance between left and right points
        leftLine = leftList[endl]
        pxLeft = leftLine.odist*math.cos(leftLine.oangle)
        pyLeft = leftLine.odist*math.sin(leftLine.oangle)
        
        rightLine = rightList[endr]
        pxRight = rightLine.odist*math.cos(rightLine.oangle)
        pyRight = rightLine.odist*math.sin(rightLine.oangle)

        xdif = pxLeft - pxRight
        ydif = pyLeft - pyRight

        gap.clearance = math.sqrt(xdif**2 + ydif**2)
       
        """
        print "debug3:", startl, startr, endl, endr 
        print "debug2:", leftLine.odist, leftLine.oangle
        print "debug2:", rightLine.odist, rightLine.oangle
        print "debug:", pxLeft, pyLeft, pxRight, pxLeft
        """

def calcClearancesAux(shortenedLidar, gapIndexes):
    scan = []

    for i in range(len(shortenedLidar)):
        isMax = (shortenedLidar[i].dist >= (MAX_VAL - MAX_VAL_PRECISION))
        scan.append(ScanLine(i, shortenedLidar[i].dist, shortenedLidar[i].angle, isMax))

    gaps = []

    for i in range(len(gapIndexes)/2):
        index = (gapIndexes[i*2] + gapIndexes[i*2 + 1])/2
        angle = shortenedLidar[index].angle
        gaps.append(Gap(index, angle, -1.0))
    
    calcClearances(scan, gaps, ROBOT_WIDTH)

    return [line.clearance for line in scan]

"""
Test code

scanLines = [
    ScanLine(0, 1.0, -math.pi/2.0, True),
    ScanLine(1, .8, -math.pi/4.0, False),
    ScanLine(2, 1.0, 0, True),
    ScanLine(3, 1.0, math.pi/4.0, True),
    ScanLine(4, 1.0, math.pi/2.0, True),
]

gapLines = [
    Gap(3, math.pi/4.0, -1.0)
]

calcClearances(scanLines, gapLines, 1.0)

print gapLines[0].clearance
"""
