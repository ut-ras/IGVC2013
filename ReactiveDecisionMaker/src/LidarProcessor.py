#!/usr/bin/env python
import math, pygame
from ReactiveUtils import *

# size of display window
SIZEX = 400
SIZEY = 400

# for displaying lidar beams
PIXELS_PER_METER = 100

class LidarValue:
    def __init__(self, dist, angle):
        self.dist = dist
        self.angle = angle

lidarValues = []

def shortenAndCorrectScan(data):
    numRanges = len(data.ranges)

    global lidarValues
    if len(lidarValues) != numRanges:
        lidarValues = [LidarValue(0.0, 0.0) for i in range(numRanges)]

    for i in range(numRanges):
        dist = data.ranges[i]
        angle = i*data.angle_increment - math.pi/2.0
        
        dist = min(dist, MAX_VAL)

        if dist < MIN_VAL:
            dist = MAX_VAL

        lidarValues[i].dist = dist
        lidarValues[i].angle = angle

    return lidarValues

window = None
background = None
center = None

def initLIDARGraphics():
    pygame.init() 
    window = pygame.display.set_mode((SIZEX, SIZEY)) 

    background = pygame.Surface(window.get_size())
    background = background.convert()
    background.fill((0, 0, 0))

    center = (SIZEX/2, SIZEY/2)

def drawLidar(heading=0):
    window.blit(background, (0,0))

    for i in range(len(lidarValues)):
        angle = boundAngleTo2PI(lidarValues[i].angle + heading)

        dist = lidarValues[i].dist
        angle = lidarValues[i].angle
        x = center[0] + PIXELS_PER_METER*dist*math.cos(angle)
        y = center[1] + PIXELS_PER_METER*dist*math.sin(angle)

        pygame.draw.line(window, (255, 255, 255), center, (x, SIZEY - y))

    pygame.display.flip() 

