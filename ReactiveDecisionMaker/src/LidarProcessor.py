#!/usr/bin/env python
import math, pygame
from ReactiveUtils import ReactiveUtils


SIZEX = 400
SIZEY = 400

MAX_VAL_THREASHOLD = 1e-3 # precision around max
MIN_VAL = 2e-2 # if ranges are below this, assume they are actually max values

DISTANCE_GUI_SCALE = 100


class LidarValue:
    def __init__(self, dist, angle):
        self.dist = dist
        self.angle = angle

class LidarProcessor:
    def __init__(self, SHOW_GRAPHICS=True):
        self.lidarValues = []
        self.SHOW_GRAPHICS = SHOW_GRAPHICS

        if self.SHOW_GRAPHICS:
            pygame.init() 
            self.window = pygame.display.set_mode((SIZEX, SIZEY)) 

            self.background = pygame.Surface(self.window.get_size())
            self.background = self.background.convert()
            self.background.fill((0, 0, 0))

            self.center = (SIZEX/2, SIZEY/2)

    def drawLidar(self):
        self.window.blit(self.background, (0,0))

        for i in range(len(self.lidarValues)):
            dist = self.lidarValues[i].dist
            angle = self.lidarValues[i].angle
            x = self.center[0] + DISTANCE_GUI_SCALE*dist*math.cos(angle)
            y = self.center[1] + DISTANCE_GUI_SCALE*dist*math.sin(angle)

            pygame.draw.line(self.window, (255, 255, 255), self.center, (x, SIZEY-y))

        pygame.display.flip() 

    def shortenAndCorrectScan(self, data, maxval):
        if len(self.lidarValues) != range(len(data.ranges)):
            self.lidarValues = [LidarValue(0,0) for i in range(len(data.ranges))]

        for i in range(len(data.ranges)):
            dist = data.ranges[i]
            angle = i*data.angle_increment - math.pi/2.0
            
            dist = min(dist, maxval)

            if dist < MIN_VAL:
                dist = maxval

            self.lidarValues[i].dist = dist
            self.lidarValues[i].angle = angle

        return self.lidarValues

    def rotateLidarValues(self, heading):
        for i in range(len(self.lidarValues)):
            self.lidarValues[i].angle = ReactiveUtils.boundAngleTo2PI(self.lidarValues[i].angle + heading)

        if self.SHOW_GRAPHICS:
            self.drawLidar();

        return self.lidarValues
            
