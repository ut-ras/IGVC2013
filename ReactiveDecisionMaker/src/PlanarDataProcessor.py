#!/usr/bin/env python
import math, pygame
from ReactiveUtils import *

# size of display window
SIZEX = 400
SIZEY = 400

# for displaying lidar beams
PIXELS_PER_METER = 100

class Ray:
    def __init__(self, dist, angle):
        self.dist = dist
        self.angle = angle

rays = []

def shortenAndCorrectPlanarData(pdata):
    numRanges = len(pdata.ranges)

    global rays
    if len(rays) != numRanges:
        rays = [Ray(0.0, 0.0) for i in range(numRanges)]

    for i in range(numRanges):
        dist = pdata.ranges[i]
        angle = pdata.angles[i]

        dist = min(dist, MAX_VAL)

        """
        These next two lines are legacy. Allow me to tell you a story.

        Back in the good old days when our Hokuyo wasn't broke; back when
        LaserScans were just that--LaserScans, and not emulated by processed
        camera images or misnamed sonar scans... back in THOSE DAYS, the
        only thing that was used in the ReactiveDecisionMaker was the hokuyo
        data. That data was made up of a bunch of ranges from the many beams
        that the Hokuyo scanned. The thing is, that was all the Hokuyo could
        give the driver--there weren't any other fields. So, in order to
        encode errors having to do with individual beams (the "out of range"
        error, for example), it reserved very small values. That means that
        if you got for a distance some very, very small number, that number
        actually instead indicated an error for that particular beam. Turns
        out the only error that isn't fatal is the "out of range" error, which
        in reality just means that the beam went all the way to the maximum
        distance. There were other possible errors, but if you got those you
        probably had bigger problems (SPOILERS: WE HAD BIGGER PROBLEMS). Ergo,
        these two lines make the assumption that if the value for a beam is
        very small, it should instead be interpreted as a maximum value.

        Sorry for bothering you with all this irrelevant information. These
        two lines are pretty useless now, and they will cause errors in the
        edge case that some fake LaserScan sincerly gives a distance that is
        very small. This case would *probably* never happen in reality, but if
        it does for an instant, oh well. Life and the universe will carry on,
        as it always has, with or without code that handles all cases in a
        perfect manner.

        Anyway, thanks for indulging me.

        -R08Z
        """
        if dist < MIN_VAL:
            dist = MAX_VAL

        rays[i].dist = dist
        rays[i].angle = angle

    return rays

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

    for i in range(len(rays)):
        angle = boundAngleTo2PI(rays[i].angle + heading)

        dist = rays[i].dist
        angle = rays[i].angle
        x = center[0] + PIXELS_PER_METER*dist*math.cos(angle)
        y = center[1] + PIXELS_PER_METER*dist*math.sin(angle)

        pygame.draw.line(window, (255, 255, 255), center, (x, SIZEY - y))

    pygame.display.flip()

