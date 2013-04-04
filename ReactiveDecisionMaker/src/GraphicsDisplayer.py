#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math, pygame


from ReactiveUtils import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

SIZEX = 400
SIZEY = 400
PIXELS_PER_METER = 100
REF_POS = (SIZEX/2, SIZEY/2)

ROTATE_90 = True
BEAM_COLOR = (255, 255, 255) # white
POS_DIR_COLOR = (0, 255, 0) # green
BEST_DIR_COLOR = (255, 0, 0) # red

class GraphicsDisplayer:
    def __init__(self):
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

            if ROTATE_90:
                angle += math.pi/2.0

            x = REF_POS[0] + PIXELS_PER_METER*dist*math.cos(angle + heading)
            y = REF_POS[1] + PIXELS_PER_METER*dist*math.sin(angle + heading)

            pygame.draw.line(self.window, BEAM_COLOR, REF_POS, (x, SIZEY - y))

        if endangles != None and enddists != None:
            for i in range(len(endangles)/2):
                index = i*2

                angle1 = endangles[index]
                angle2 = endangles[index+1]
                dist1 = enddists[index]
                dist2 = enddists[index+1]

                if ROTATE_90:
                    angle1 += math.pi/2.0
                    angle2 += math.pi/2.0

                x1 = REF_POS[0] + PIXELS_PER_METER*dist1*math.cos(angle1 + heading)
                y1 = REF_POS[1] + PIXELS_PER_METER*dist1*math.sin(angle1 + heading)
                x2 = REF_POS[0] + PIXELS_PER_METER*dist2*math.cos(angle2 + heading)
                y2 = REF_POS[1] + PIXELS_PER_METER*dist2*math.sin(angle2 + heading)

                pygame.draw.circle(self.window, (0, 0, 255), (int(x1), int(SIZEY - y1)), 5, 5)
                pygame.draw.circle(self.window, (0, 255, 0), (int(x2), int(SIZEY - y2)), 5, 5)

        for i in range(len(viableDirections)):
            direction = viableDirections[i].direction

            if ROTATE_90:
                direction += math.pi/2.0

            pygame.draw.line(
                self.window,
                POS_DIR_COLOR,
                REF_POS,
                (REF_POS[0] + 100*math.cos(direction),
                SIZEY - (REF_POS[1] + 100*math.sin(direction))),
                4
            )

        if None != bestDirection:
            direction = bestDirection.direction

            if ROTATE_90:
                direction += math.pi/2.0

            pygame.draw.line(
                self.window,
                BEST_DIR_COLOR,
                REF_POS,
                (REF_POS[0] + 100*math.cos(direction),
                SIZEY - (REF_POS[1] + 100*math.sin(direction))),
                4
            )

        pygame.display.flip()




