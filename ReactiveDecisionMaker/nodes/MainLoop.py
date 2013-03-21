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

    directionFinder = DirectionFinder(
        True, 
        MIN_CLEARANCE_ALLOWED=ReactiveUtils.MIN_CLEARANCE
    )

    lidarProcessor = LidarProcessor(False)

    goalCalculator = GoalCalculator(
        MIN_CLEARANCE_ALLOWED=.1
    )

    directionChooser = DirectionChooser(ReactiveUtils.MAX_VAL*2)
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
            print 'got invalid data from GetScan service...',\
                  'is anything being published to /scan?'
            continue

        distToGoal = ReactiveUtils.euclidDistPoint(goalPos, curPos)

        print distToGoal

        if distToGoal < ReactiveUtils.CLOSE_ENOUGH_TO_GOAL:
            print "stopping because we're close enough to the goal"

            msg = Twist()
            pub.publish(msg)

            r.sleep()
            continue

        shortenedLidar = lidarProcessor.shortenAndCorrectScan(
            scan, 
            ReactiveUtils.MAX_VAL
        )

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
            bestDirection = directionChooser.pickBestDirection(
                directions, 
                goalHeading, 
                heading
            )

            msg = directionFollower.getAction(
                bestDirection.direction, 
                heading
            )

            pub.publish(msg)

        graphicsDisplayer.drawEverything(
            shortenedLidar,
            heading,
            directions,
            directionFinder.endangles,
            directionFinder.enddists,
            bestDirection
        )

        r.sleep()







