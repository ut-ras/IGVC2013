#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math, pygame

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from ReactiveUtils import ReactiveUtils


MAX_VAL_THREASHOLD = 1e-3 # precision around max


class GraphicsDisplayer:
    def __init__(self, SIZEX=400, SIZEY=400, DISTANCE_GUI_SCALE=100):
        self.SIZEX = SIZEX
        self.SIZEY = SIZEY
        self.DISTANCE_GUI_SCALE = DISTANCE_GUI_SCALE

        self.refPos = (SIZEX/2, SIZEY/2)

        pygame.init()
        self.window = pygame.display.set_mode((SIZEX, SIZEY))
        self.background = pygame.Surface(self.window.get_size())
        self.background = self.background.convert()
        self.background.fill((0, 0, 0))

    def drawEverything(self, shortenedLidar, heading, viableDirections, endangles, enddists, bestDirection):
        self.window.blit(self.background, (0,0))

        for i in range(len(shortenedLidar)) :
            angle = shortenedLidar[i].angle
            dist = shortenedLidar[i].dist

            angle += math.pi/2

            x = self.refPos[0] + self.DISTANCE_GUI_SCALE*dist*math.cos(angle + heading)
            y = self.refPos[1] + self.DISTANCE_GUI_SCALE*dist*math.sin(angle + heading)

            pygame.draw.line(self.window, (255, 255, 255), self.refPos, (x, self.SIZEY - y))

        for i in range(len(endangles)/2):
            index = i*2

            angle1 = endangles[index]
            angle2 = endangles[index+1]
            dist1 = enddists[index]
            dist2 = enddists[index+1]

            angle1 += math.pi/2
            angle2 += math.pi/2

            x1 = self.refPos[0] + self.DISTANCE_GUI_SCALE*dist1*math.cos(angle1 + heading)
            y1 = self.refPos[1] + self.DISTANCE_GUI_SCALE*dist1*math.sin(angle1 + heading)
            x2 = self.refPos[0] + self.DISTANCE_GUI_SCALE*dist2*math.cos(angle2 + heading)
            y2 = self.refPos[1] + self.DISTANCE_GUI_SCALE*dist2*math.sin(angle2 + heading)

            pygame.draw.circle(self.window, (0, 0, 255), (int(x1), int(self.SIZEY - y1)), 5, 5)
            pygame.draw.circle(self.window, (0, 255, 0), (int(x2), int(self.SIZEY - y2)), 5, 5)

        for i in range(len(viableDirections)):
            direction = viableDirections[i].direction
            clearance = viableDirections[i].clearance

            direction += math.pi/2

            pygame.draw.line(
                self.window,
                (0, 255, 255),
                self.refPos,
                (self.refPos[0] + 100*math.cos(direction),
                 self.SIZEY - (self.refPos[1] + 100*math.sin(direction))),
                4
            )

        if bestDirection != None:
            direction = bestDirection.direction
            clearance = bestDirection.clearance

            direction += math.pi/2

            pygame.draw.line(
                self.window,
                (255, 0, 0),
                self.refPos,
                (self.refPos[0] + 100*math.cos(direction),
                 self.SIZEY - (self.refPos[1] + 100*math.sin(direction))),
                4
            )

        pygame.display.flip()




