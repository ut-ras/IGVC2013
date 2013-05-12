#!/usr/bin/env python

import math, random, pygame, time, sys
from threading import Thread
from pygame.locals import *

from Constants import *
from Geometry import GLib
from ClearanceCalculator import CLib
from Plotter import Plotter
from DynamicWindow import calcDynamicWindow

DT = .1
NUM_OBSTACLES = 200

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 500

class CloudPoint:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

def plotEverything():
    plotter.clear()

    plotter.plotPoint(cur_x, cur_y, CLEARANCE_MAX, "lightBlue")
    plotter.plotPoint(cur_x, cur_y, SIZE_RADIUS, "lightGreen")
    plotter.plotCircle(cur_x, cur_y, SIZE_RADIUS, "darkGreen")
    plotter.drawAxises(1, 1)

    for cloudPoint in cloud:
        plotter.plotPoint(cloudPoint.x, cloudPoint.y, 0.05, "black")
        plotter.plotCircle(cloudPoint.x, cloudPoint.y, cloudPoint.r, "gray")

    plotter.plotPoint(goal_x, goal_y, 0.1, "blue")

def step():
    global cur_x, cur_y, cur_dir, cur_lin, cur_ang, goal_x, goal_y

    plotEverything()

    mycloud = []
    for cloudPoint in cloud:
        if GLib.euclid(cloudPoint.x, cloudPoint.y, cur_x, cur_y) < CLEARANCE_MAX:
            mycloud.append(cloudPoint);

    (cur_ang, cur_lin) = calcDynamicWindow(
        cur_x,
        cur_y,
        cur_dir,
        cur_lin,
        cur_ang,
        mycloud,
        goal_x,
        goal_y,
        DT,
        plotter
        )

    kine = GLib.calcTrajectoryStepFromTime(
        cur_x,
        cur_y,
        cur_dir,
        cur_lin,
        cur_ang,
        DT
        )

    cur_x = kine[0]
    cur_y = kine[1]
    cur_dir = kine[2]

    plotter.plotArrow(cur_x, cur_y, cur_dir, "red")
    plotter.display()

def loopPlotter():
    while True:
        step()
        time.sleep(.01)

def onmousedown(x, y):
    global goal_x, goal_y

    coords = plotter.getPlotCoords(x, y)
    goal_x = coords[0]
    goal_y = coords[1]

cur_x = 0
cur_y = 0
cur_dir = 0
goal_x = 5
goal_y = 5
cur_lin = 0
cur_ang = 0
cloud = []

if __name__ == '__main__':
    print "everything compiles!"

    for i in range(NUM_OBSTACLES):
        while True:
            x = random.random()*26-13
            y = random.random()*14-7
            if GLib.euclid(x, y, cur_x, cur_y) > 2.0*SIZE_RADIUS:
                break

        cloud.append(CloudPoint(
            x,
            y,
            SIZE_RADIUS + BUFFER_SPACE
            ))

    pygame.init()
    window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    plotter = Plotter(window, WINDOW_WIDTH, WINDOW_HEIGHT, .5, .5, 50, 50)
    plotEverything()

    Thread(target=loopPlotter, args=[]).start()

    while True:
        for event in pygame.event.get():
            print event
            if event.type == QUIT:
                pygame.quit()
                sys.exit("you hit the x-button!");
            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    onmousedown(event.pos[0], event.pos[1])




