#!/usr/bin/env python
import rospy, math, random, pygame, time, sys

from threading import Thread
from pygame.locals import *

from Constants import *
from Geometry import GLib
from ClearanceCalculator import CLib
from Plotter import Plotter
from DynamicWindow import calcDynamicWindow

import roslib; roslib.load_manifest('DynamicWindow')

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Twist, Point

NUM_OBSTACLES = 200

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 500

RATE = 10.0 # Hz

class CloudPoint:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

class DecisionMaker:
    def __init__(self):
        pass

    def initServices(self):
        rospy.wait_for_service('getPos')
        self.getPos = rospy.ServiceProxy('getPos', GetPos)

        rospy.wait_for_service('getHeading')
        self.getHeading = rospy.ServiceProxy('getHeading', GetHeading)

        rospy.wait_for_service('getPlanarData')
        self.getPlanarData = rospy.ServiceProxy('getPlanarData', GetPlanarData)

        rospy.wait_for_service('getGoal')
        self.getGoal = rospy.ServiceProxy('getGoal', GetGoal)

        rospy.wait_for_service('getTwist')
        self.getTwist = rospy.ServiceProxy('getTwist', GetTwist)

        # these three variables are for turning around in case we hit a deadend
        self.turningAround = False
        self.turningDir = 0
        self.oppositeAngle = None # would this be a good name for a band?

    def acquireData(self):
        try:
            self.curPos = self.getPos().pos
            self.heading = self.getHeading().heading
            self.pdata = self.getPlanarData().pdata
            self.goalPos = self.getGoal().goal
            self.curTwist = self.getTwist().twist
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def processData(self):
        # move goal to our reference frame
        x = self.goalPos.x - self.curPos.x
        y = self.goalPos.y - self.curPos.y
        t = self.heading
        (self.goalPos.x, self.goalPos.y) = (
            x*math.cos(t) - y*math.sin(t),
            x*math.sin(t) + y*math.cos(t)
            )

        # reset out pos & heading to be the center of the reference frame
        # (otherwise we wouldn't alter the goal, and we'd have to convert all of the points)
        self.curPos.x = 0
        self.curPos.y = 0
        self.heading = 0

        # conver pdata to cloud
        self.cloud = []
        for i in range(len(self.pdata.ranges)):
            dist = self.pdata.ranges[i]
            angle = self.pdata.angles[i]

            if dist < CLEARANCE_MAX:
                self.cloud.append(CloudPoint(
                    dist*math.cos(angle),
                    dist*math.sin(angle),
                    SIZE_RADIUS + BUFFER_SPACE
                    ))

    def iterate(self):
        curPos = self.curPos
        heading = self.heading
        cloud = self.cloud
        goalPos = self.goalPos
        curTwist = self.curTwist

        msg = Twist()

        if len(cloud) == 0:
            print 'got invalid data from GetPlanarData service...',\
                  'so um is anything being published to /planar_data?'
            return msg

        if goalPos.z == TIMEOUT_ERROR:
            # this indicates a goal data timeout in the DataServiceProvider
            # making us stop if GoalMaker has stopped publishing to /goal
            rospy.loginfo("stopping because of a goal data timeout")
            return msg

        distToGoal = GLib.euclid(goalPos.x, goalPos.y, curPos.x, curPos.y)

        # print "distance to goal:", distToGoal

        if distToGoal < CLOSE_ENOUGH_TO_GOAL:
            rospy.loginfo("stopping because we're close enough to the goal")
            return msg

        (msg.linear.x, msg.angular.z) = calcDynamicWindow(
            curPos.x,
            curPos.y,
            heading,
            curTwist.linear.x,
            curTwist.angular.z,
            cloud,
            goalPos.x,
            goalPos.y,
            1/RATE,
            plotter
            )

        plotEverything(
            curPos.x,
            curPos.y,
            heading,
            goalPos.x,
            goalPos.y,
            cloud
            )

        return msg

def plotEverything(cur_x, cur_y, cur_dir, goal_x, goal_y, cloud):
    plotter.clear()

    plotter.plotPoint(cur_x, cur_y, CLEARANCE_MAX, "lightBlue")
    plotter.plotPoint(cur_x, cur_y, SIZE_RADIUS, "lightGreen")
    plotter.plotCircle(cur_x, cur_y, SIZE_RADIUS, "darkGreen")
    plotter.drawAxises(1, 1)

    for cloudPoint in cloud:
        plotter.plotPoint(cloudPoint.x, cloudPoint.y, 0.05, "black")
        plotter.plotCircle(cloudPoint.x, cloudPoint.y, cloudPoint.r, "gray")

    plotter.plotPoint(goal_x, goal_y, 0.1, "blue")
    plotter.plotArrow(cur_x, cur_y, cur_dir, "red")

    plotter.display()

TIMEOUT = 1.0 # seconds
latestAction = None
latestTime = None

def handle_getAction(req):
    if latestAction == None:
        return GetActionResponse(Twist())

    curTime = rospy.get_time()

    if curTime - latestTime < TIMEOUT:
        return GetActionResponse(latestAction)
    else:
        rospy.loginfo("last action stale; GetAction service giving stop action")
        return GetActionResponse(Twist())

if __name__ == "__main__":
    pygame.init()
    window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    plotter = Plotter(window, WINDOW_WIDTH, WINDOW_HEIGHT, .5, .5, 50, 50)

    rospy.init_node('DWMainLoop')

    serv = rospy.Service('getAction', GetAction, handle_getAction)

    decisionMaker = DecisionMaker()
    decisionMaker.initServices()

    r = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        r.sleep()

        success = decisionMaker.acquireData()

        print success

        if success:
            decisionMaker.processData()
            msg = decisionMaker.iterate()
            latestTime = rospy.get_time()
            latestAction = msg
