#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math, pygame

from sensor_msgs.msg import LaserScan

SIZEX = 400
SIZEY = 400

MAX_VAL_THREASHOLD = 1e-3 # precision around max

DISTANCE_GUI_SCALE = 100


class Direction:
    def __init__(self, direction, clearance):
        self.direction = direction
        self.clearance = clearance


def euclidDist2D(p0, p1):
    return math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

def calcClearance(curPos, angle1, dist1, angle2, dist2) :
    dist = min(dist1, dist2)

    p0 = (curPos[0] + dist*math.cos(angle1),\
	      curPos[1] + dist*math.sin(angle1))
    p1 = (curPos[0] + dist*math.cos(angle2),\
	      curPos[1] + dist*math.sin(angle2))

    return euclidDist2D(p0, p1)
	
def averageAngle(angle1, angle2):
    xcomp = (math.cos(angle1) + math.cos(angle2))/2.0
    ycomp = (math.sin(angle1) + math.sin(angle2))/2.0
	
    return math.atan2(ycomp, xcomp)

pi2 = math.pi*2
def boundAngleTo2PI(angle):
    return (angle%pi2 + pi2)%pi2;


class DirectionFinder:
    def __init__(self, SHOW_GRAPHICS=True, MAX_VAL=1, MIN_CLEARANCE_ALLOWED=.61):
        self.SHOW_GRAPHICS = SHOW_GRAPHICS
        self.MAX_VAL = MAX_VAL
        self.MIN_CLEARANCE_ALLOWED = MIN_CLEARANCE_ALLOWED
        self.curPos = (SIZEX/2, SIZEY/2)

        self.endangles = None
        self.enddists = None
        self.endpoints = None
        self.viableDirections = None
        self.bestDirection = None

        if self.SHOW_GRAPHICS:
            pygame.init() 
            self.window = pygame.display.set_mode((SIZEX, SIZEY)) 

            self.background = pygame.Surface(self.window.get_size())
            self.background = self.background.convert()
            self.background.fill((0, 0, 0))

    def drawEverything(self, shortenedLidar, heading):
        self.window.blit(self.background, (0,0))

        for i in range(len(shortenedLidar)) :
            angle = shortenedLidar[i].angle
            dist = shortenedLidar[i].dist

            x = self.curPos[0] + DISTANCE_GUI_SCALE*dist*math.cos(angle + heading)
            y = self.curPos[1] + DISTANCE_GUI_SCALE*dist*math.sin(angle + heading)

            pygame.draw.line(self.window, (255, 255, 255), self.curPos, (x, SIZEY-y))

        for i in range(len(self.endangles)/2):
            index = i*2

            angle1 = self.endangles[index]
            angle2 = self.endangles[index+1]
            dist1 = self.enddists[index]
            dist2 = self.enddists[index+1]

            x1 = self.curPos[0] + DISTANCE_GUI_SCALE*dist1*math.cos(angle1 + heading)
            y1 = self.curPos[1] + DISTANCE_GUI_SCALE*dist1*math.sin(angle1 + heading)
            x2 = self.curPos[0] + DISTANCE_GUI_SCALE*dist2*math.cos(angle2 + heading)
            y2 = self.curPos[1] + DISTANCE_GUI_SCALE*dist2*math.sin(angle2 + heading)

            pygame.draw.circle(self.window, (0, 0, 255), (int(x1),int(SIZEY-y1)), 5, 5)
            pygame.draw.circle(self.window, (0, 255, 0), (int(x2),int(SIZEY-y2)), 5, 5)

        for i in range(len(self.viableDirections)):
            direction = self.viableDirections[i].direction
            clearance = self.viableDirections[i].clearance

            pygame.draw.line(self.window, (0, 255, 255), self.curPos, \
                (self.curPos[0] + 100*math.cos(direction), \
                 SIZEY-(self.curPos[1] + 100*math.sin(direction))), 4)

        if self.bestDirection != None:
            direction = self.bestDirection.direction
            clearance = self.bestDirection.clearance

            pygame.draw.line(self.window, (255, 0, 0), self.curPos, \
                (self.curPos[0] + 100*math.cos(direction), \
                 SIZEY-(self.curPos[1] + 100*math.sin(direction))), 4)

        pygame.display.flip() 

    def getViableDirections(self, shortenedLidar):
        self.endangles = []
        self.enddists = []
        self.endpoints = []
        self.viableDirections = []

        startGap = False

        for i in range(len(shortenedLidar)) :
            angle = shortenedLidar[i].angle
            dist = shortenedLidar[i].dist
            
            x = self.curPos[0] + DISTANCE_GUI_SCALE*dist*math.cos(angle)
            y = self.curPos[1] + DISTANCE_GUI_SCALE*dist*math.sin(angle)
               
            isMaxVal = abs(dist - self.MAX_VAL) < MAX_VAL_THREASHOLD

            if startGap and ((not isMaxVal) or i == len(shortenedLidar) - 1):
                startGap = False
                self.endangles.append(angle)
                self.enddists.append(dist)
                self.endpoints.append((int(x), int(y)))
            elif (not startGap) and isMaxVal and i != len(shortenedLidar) - 1:
                startGap = True;
                self.endangles.append(angle)
                self.enddists.append(dist)
                self.endpoints.append((int(x), int(y)))

        for i in range(len(self.endpoints)/2):
            index = i*2

            direction = averageAngle(self.endangles[index], self.endangles[index+1])
            clearance = calcClearance(self.curPos, self.endangles[index], 
                self.enddists[index], self.endangles[index+1], self.enddists[index+1])

            if clearance > self.MIN_CLEARANCE_ALLOWED:
                self.viableDirections.append(Direction(direction, clearance))

        return self.viableDirections

    def rotateDirections(self, viableDirections, heading):
        for i in range(len(viableDirections)):
            viableDirections[i].direction = \
                boundAngleTo2PI(viableDirections[i].direction + heading)

