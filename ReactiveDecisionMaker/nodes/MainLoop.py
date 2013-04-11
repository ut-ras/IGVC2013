#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, random, math

from ReactiveUtils import *

from DirectionFinder import calcViableDirs
from PlanarDataProcessor import shortenAndCorrectPlanarData
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

        rospy.wait_for_service('getPlanarData')
        self.getPlanarData = rospy.ServiceProxy('getPlanarData', GetPlanarData)

        rospy.wait_for_service('getGoal')
        self.getGoal = rospy.ServiceProxy('getGoal', GetGoal)

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
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def iterate(self):
        curPos = self.curPos
        heading = self.heading
        pdata = self.pdata
        goalPos = self.goalPos

        msg = Twist()

        if len(pdata.ranges) == 0:
            print 'got invalid data from GetPlanarData service...',\
                  'so um is anything being published to /planar_data?'
            return msg

        if goalPos.z == TIMEOUT_ERROR:
            # this indicates a goal data timeout in the DataServiceProvider
            # making us stop if GoalMaker has stopped publishing to /goal
            rospy.loginfo("stopping because of a goal data timeout")
            return msg

        distToGoal = euclidDistPoint(goalPos, curPos)

        # print "distance to goal:", distToGoal

        if distToGoal < CLOSE_ENOUGH_TO_GOAL:
            rospy.loginfo("stopping because we're close enough to the goal")
            return msg

        shortenedLidar = shortenAndCorrectPlanarData(pdata)

        directions = calcViableDirs(shortenedLidar)
        rotateDirections(directions, heading)

        goalHeading = calcGoalHeading(curPos, goalPos)
        goalDirection = calcViableDir(
                goalHeading,
                shortenedLidar,
                heading,
                curPos,
                goalPos,
                pdata.startAngle,
                pdata.angleRange
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

            # print "best direction: ", bestDirection

            if bestDirection != None:
                msg = getAction(
                        bestDirection.direction,
                        heading,
                        pdata.angleRange
                        )

            self.turningAround = False
            self.oppositeAngle = None
        else:
            # if there are no viable directions, turn until there are
            ### if we already decided to turn around before, keeping turning
            if self.turningAround:
                if abs(heading - self.oppositeAngle) < ANGLE_PRECISION:
                    self.turningAround = False
                    self.turningDir = 0
                elif 1 == self.turningDir:
                    # turn left
                    msg.angular.z = MAX_ANGULAR
                elif 2 == self.turningDir:
                    # turn right
                    msg.angular.z = -MAX_ANGULAR
            ### if we're not already turning, decided which direction to go
            else:
                self.turningAround = True

                coin = random.random()
                if coin < .5:
                    self.oppositeAngle = boundAngleTo2PI(heading - ANGLE_PRECISION)
                    self.turningDir = 1;
                    # turn left
                    msg.angular.z = MAX_ANGULAR
                elif coin >= .5:
                    self.oppositeAngle = boundAngleTo2PI(heading + ANGLE_PRECISION)
                    self.turningDir = 2
                    # turn right
                    msg.angular.z = -MAX_ANGULAR

        self.graphicsDisplayer.drawEverything(
                shortenedLidar,
                heading,
                directions,
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
        return GetActionResponse(latestAction)
    else:
        rospy.loginfo("last action stale; GetAction service giving stop action")
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
