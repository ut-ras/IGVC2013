#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math

from ReactiveUtils import *

from DirectionFinder import getViableDirections, getEndangles, getEnddists
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

        directions = getViableDirections(shortenedLidar)
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

            msg = getAction(
                bestDirection.direction, 
                heading
            )

        self.graphicsDisplayer.drawEverything(
            shortenedLidar,
            heading,
            directions,
            getEndangles(),
            getEnddists(),
            bestDirection
        )

        return msg


if __name__ == "__main__":
    rospy.init_node('MainReactiveLoop')
    pub = rospy.Publisher('vel_cmd', Twist)

    decisionMaker = DecisionMaker()
    decisionMaker.initServices()

    r = rospy.Rate(10) 

    while not rospy.is_shutdown():
        r.sleep()
        
        success = decisionMaker.acquireData()

        if success:
            msg = decisionMaker.iterate()
            pub.publish(msg)
