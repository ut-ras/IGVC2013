#!/usr/bin/env python
import math, random, pygame, time
from threading import Thread
from pygame.locals import *

from Geometry import GLib
from ClearanceCalculator import CLib
from Plotter import Plotter
from Constants import *

NUM_ANGULARS = int((ANGULAR_MAX - ANGULAR_MIN)/ANGULAR_INC)
NUM_LINEARS = int((LINEAR_MAX - LINEAR_MIN)/LINEAR_INC)
angularArr = [x*ANGULAR_INC + ANGULAR_MIN for x in range(NUM_ANGULARS)]
linearArr = [x*LINEAR_INC + LINEAR_MIN for x in range(NUM_LINEARS)]

DO_NOTHING = (0,0)

# returns tuple of velocities: (angular, linear)
def calcDynamicWindow(
    cur_x,
    cur_y,
    cur_dir,
    cur_linear,
    cur_angular,
    mycloud,
    goal_x,
    goal_y,
    deltaTime,
    plotter=None
    ):

    distToGoal = GLib.euclid(goal_x, goal_y, cur_x, cur_y)

    if distToGoal < CLOSE_ENOUGH_TO_GOAL:
        return DO_NOTHING

    best_weight = -1
    best_linear = None
    best_angular = None
    best_found = False

    for angular in angularArr:
        if angular < cur_angular - ANGULAR_ACCEL or \
           angular > cur_angular + ANGULAR_ACCEL:
            continue

        for linear in linearArr:
            if linear < cur_linear - LINEAR_ACCEL or \
               linear > cur_linear + LINEAR_ACCEL:
                continue

            #
            # get clearance and normalize
            #
            res = CLib.calcIntersection(
                cur_x,
                cur_y,
                cur_dir,
                linear,
                angular,
                mycloud
                )

            clearance = CLEARANCE_MAX
            color = "gray"

            if res.point and res.delta < CLEARANCE_MAX:
                clearance = res.delta
                color = "red"

                if clearance < CLEARANCE_MIN:
                    continue

            clearanceNorm = clearance/CLEARANCE_MAX

            #
            # get normalized goal direction weight
            #
            newDir = cur_dir + angular*deltaTime
            goalDir = math.atan2(goal_y - cur_y, goal_x - cur_x)
            goalDirDif = math.pi - abs(GLib.angleDif(goalDir, newDir))
            goalDirDifNorm = goalDirDif/math.pi

            #
            # get normalized linear velocity weight
            #
            linearNorm = linear/LINEAR_MAX

            #
            # compute the long-term goal proximity weight
            #
            if res.traj.type == "vector":
                goaldist = abs(goal_y)
            elif res.traj.type == "circle":
                gd = GLib.euclid(goal_x, goal_y, res.traj.x, res.traj.y)
                goaldist = abs(gd - res.traj.r)
            elif res.traj.type == "point":
                goaldist = GLib.euclid(goal_x, goal_y, cur_x, cur_y)

            goaldist = GOAL_DIST_MAX if goaldist > GOAL_DIST_MAX else goaldist
            goalDistNorm = goaldist/GOAL_DIST_MAX

            #
            # get the final weight, compare it with what we've seen so far
            #
            weight = clearanceNorm*CLEARANCE_WEIGHT + \
                         goalDirDifNorm*GOAL_DIR_WEIGHT + \
                         linearNorm*LINEAR_WEIGHT + \
                         goalDistNorm*GOAL_DIST_WEIGHT

            if weight > best_weight:
                best_weight = weight
                best_linear = linear
                best_angular = angular
                best_found = True

            #
            # plot trajectory & intersection point, if there was one
            #
            if plotter:
                if res.point:
                    plotter.plotPoint(res.point.x, res.point.y, .1, color)

                if res.traj.type == "circle":
                    plotter.plotCircle(res.traj.x, res.traj.y, res.traj.r, color)
                elif res.traj.type == "vector":
                    plotter.plotVector(res.traj.x, res.traj.y, res.traj.dir, color)

                plotter.display()

    if not best_found or (abs(best_linear) <= 1e-6 and abs(best_angular) <= 1e-6):
        return DO_NOTHING

    if distToGoal < SLOW_DOWN_RADIUS:
        best_linear *= (distToGoal/SLOW_DOWN_RADIUS)

    return (best_angular, best_linear)

