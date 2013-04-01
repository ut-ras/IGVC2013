#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, random, math

from ReactiveUtils import *

from DirectionFinder import calcViableDirs, getEndangles, getEnddists
from LidarProcessor import shortenAndCorrectScan
from DirectionChooser import pickBestDirection
from GoalCalculator import calcGoalHeading, calcViableDir
from DirectionFollower import getAction
from GraphicsDisplayer import GraphicsDisplayer

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Twist, Point

class DecisionMaker:
    def __init__(self):
        self.graphicsDisplayer = GraphicsDisplayer()

    def initServices(self):
        rospy.wait_for_service('getPos')
        self.getPos = rospy.ServiceProxy('getPos', GetPos)

        rospy.wait_for_service('getHeading')
        self.getHeading = rospy.ServiceProxy('getHeading', GetHeading)

        rospy.wait_for_service('getScan')
        self.getScan = rospy.ServiceProxy('getScan', GetScan)

        rospy.wait_for_service('getGoal')
        self.getGoal = rospy.ServiceProxy('getGoal', GetGoal)

        self.turningAround = False
        self.turningDir = 0
        self.oppositeAngle = None

    def acquireData(self):
        try:
            self.curPos = self.getPos().pos
            self.heading = self.getHeading().heading
            self.scan = self.getScan().scan
            self.goalPos = self.getGoal().goal
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def iterate(self):
        curPos = self.curPos
        heading = self.heading
        scan = self.scan
        goalPos = self.goalPos
        turningAround = self.turningAround
        turningDir = self.turningDir
        oppositeAngle = self.oppositeAngle # would this be a good name for a band?

        msg = Twist()

        if len(scan.ranges) == 0:
            print 'got invalid data from GetScan service...',\
                  'so um is anything being published to /scan?'
            return msg

        distToGoal = euclidDistPoint(goalPos, curPos)

        print "distance to goal:", distToGoal

        if distToGoal < CLOSE_ENOUGH_TO_GOAL:
            print "stopping because we're close enough to the goal"

            return msg

        shortenedLidar = shortenAndCorrectScan(scan)

        directions = calcViableDirs(shortenedLidar)
        print directions

        rotateDirections(directions, heading)

        goalHeading = calcGoalHeading(curPos, goalPos)
        goalDirection = calcViableDir(
            goalHeading,
            shortenedLidar,
            heading,
            curPos,
            goalPos
        )

        if goalDirection != None:
            directions.append(goalDirection)

        bestDirection = None

        if len(directions) > 0:
            bestDirection = pickBestDirection(
                directions,
                goalHeading,
                heading
            )

            if bestDirection != None:
                msg = getAction(
                    bestDirection.direction,
                    heading
                )

            turningAround = False
            oppositeAngle = None
        else:
            # if there are no viable directions, turn until there are
            if turningAround:
                if abs(heading - oppositeAngle) < ANGLE_PRECISION:
                    turningAround = False
                    turningDir = 0
                elif 1 == turningDir:
                    # turn left
                    msg.angular.z = MAX_ANGULAR
                elif 2 == turningDir:
                    # turn right
                    msg.angular.z = -MAX_ANGULAR
            else:
                turningAround = True

                coin = random.random()
                if coin < .5:
                    oppositeAngle = boundAngleTo2PI(heading - ANGLE_PRECISION)
                    turningDir = 1;
                    # turn left
                    msg.angular.z = MAX_ANGULAR
                elif coin >= .5:
                    oppositeAngle = boundAngleTo2PI(heading + ANGLE_PRECISION)
                    turningDir = 2
                    # turn right
                    msg.angular.z = -MAX_ANGULAR

        self.graphicsDisplayer.drawEverything(
            shortenedLidar,
            heading,
            directions,
            getEndangles(),
            getEnddists(),
            bestDirection
        )

        return msg

TIMEOUT = 1.0 # seconds
latestAction = None
latestTime = None

def handle_getAction(req):
    if latestAction == None:
        return GetActionResponse(Twist())

    curTime = rospy.get_time()

    if curTime - latestTime < TIMEOUT:
        print latestAction
        return GetActionResponse(latestAction)
    else:
        print 'data became too old!'
        return GetActionResponse(Twist())

if __name__ == "__main__":
    rospy.init_node('MainReactiveLoop')

    serv = rospy.Service('getAction', GetAction, handle_getAction)

    decisionMaker = DecisionMaker()
    decisionMaker.initServices()

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()

        success = decisionMaker.acquireData()

        if success:
            msg = decisionMaker.iterate()
            latestTime = rospy.get_time()
            latestAction = msg
