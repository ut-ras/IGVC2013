#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy, math

from DirectionFinder import DirectionFinder

from LidarProcessor import LidarProcessor
from GoalCalculator import GoalCalculator
from DirectionChooser import DirectionChooser
from DirectionFollower import DirectionFollower
from GraphicsDisplayer import GraphicsDisplayer
from ReactiveUtils import ReactiveUtils

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Twist, Point


MAX_VAL = 1 # in meters
CLOSE_ENOUGH_TO_GOAL = .1
MIN_CLEARANCE = 1


def euclidDist2D(p0, p1):
    return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

if __name__ == "__main__":
    rospy.init_node('MainReactiveLoop')
    pub = rospy.Publisher('vel_cmd', Twist)

    rospy.wait_for_service('getPos')
    getPos = rospy.ServiceProxy('getPos', GetPos)

    rospy.wait_for_service('getHeading')
    getHeading = rospy.ServiceProxy('getHeading', GetHeading)

    rospy.wait_for_service('getScan')
    getScan = rospy.ServiceProxy('getScan', GetScan)

    rospy.wait_for_service('getGoal')
    getGoal = rospy.ServiceProxy('getGoal', GetGoal)

    directionFinder = DirectionFinder(True, MIN_CLEARANCE_ALLOWED=MIN_CLEARANCE)
    lidarProcessor = LidarProcessor(False)
    goalCalculator = GoalCalculator(MIN_CLEARANCE_ALLOWED=MIN_CLEARANCE)
    directionChooser = DirectionChooser(MAX_VAL*2)
    directionFollower = DirectionFollower()
    graphicsDisplayer = GraphicsDisplayer()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            curPos = getPos().pos
            heading = getHeading().heading
            scan = getScan().scan
            goalPos = getGoal().goal
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if len(scan.ranges) == 0:
            print 'got invalid data from GetScan service... is anything being published to /scan?'
            continue

        if euclidDist2D(goalPos, curPos) < CLOSE_ENOUGH_TO_GOAL:
            print 'close enough to goal'
            r.sleep()
            continue

        shortenedLidar = lidarProcessor.shortenAndCorrectScan(scan, MAX_VAL);

        directions = directionFinder.getViableDirections(shortenedLidar)
        directionFinder.rotateDirections(directions, heading)

        goalHeading = goalCalculator.calcGoalHeading(curPos, goalPos)
        goalDirection = goalCalculator.calcViableDir(
            goalHeading, 
            shortenedLidar, 
            heading,
            curPos, 
            goalPos
        )

        if goalDirection != None:
            directions.append(goalDirection)

        if len(directions) > 0:
            bestDirection = directionChooser.pickBestDirection(directions, goalHeading, heading)
            msg = directionFollower.getAction(bestDirection.direction, heading)
            # pub.publish(msg)

        graphicsDisplayer.drawEverything(
            shortenedLidar, 
            heading, 
            directions,
            directionFinder.endangles, 
            directionFinder.enddists,
            bestDirection
        )

        r.sleep()







