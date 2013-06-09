#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from ReactiveUtils import *

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Point

import sys

WAYPOINT_FILENAME = '/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/offsets'

curGoalIndex = 0

pub = None

GUIDANCE_ACCURACY = 2.0

class Waypoint:
    def __init__(self, point, accur):
        self.point = point
        self.accur = accur

    def __str__(self):
        return str(self.point) + ":" + str(self.accur)

goals = [
    Waypoint(Point(20, 0, 0), CLOSE_ENOUGH_TO_GOAL),
    Waypoint(Point(0, 0, 0), CLOSE_ENOUGH_TO_GOAL)
    ]

def processPos(pos):
    global curGoalIndex

    if curGoalIndex < len(goals):
        pos = Point(pos.x, pos.y, 0)

        distToGoal = euclidDistPoint(pos, goals[curGoalIndex].point)

        if distToGoal < goals[curGoalIndex].accur:
            print "close enough to ", goals[curGoalIndex].point.x, ",", goals[curGoalIndex].point.y
            curGoalIndex += 1
            if curGoalIndex < len(goals):
                print "setting new goal to ", goals[curGoalIndex].point.x, ",", goals[curGoalIndex].point.y

        if curGoalIndex < len(goals):
            print "publishing goal ", goals[curGoalIndex]
            pub.publish(goals[curGoalIndex].point)

def readWaypoints(filename):
    print 'reading waypoints'

    try:
        f = open(filename, 'r')
    except:
        rospy.logwarn('no file found named %s', filename)
        return

    list = []

    for line in f:
        s = line.split()

        try:
            x = float(s[0])
            y = float(s[1])
            # D for dance, G for guidance
            if s[2] == 'D':
                accur = CLOSE_ENOUGH_TO_GOAL
            elif s[2] == 'G':
                accur = GUIDANCE_ACCURACY
            else:
                rospy.logwarn('error: do not recognize waypoint description: %s', line)
                return
        except:
            rospy.logwarn('error processing line: %s', line)
            return

        list.append(Waypoint(Point(x, y, 0), accur));

    return list

if __name__ == "__main__":
    try:
        if bool(rospy.get_param('GoalMaker/usefile')):
            goals = readWaypoints(WAYPOINT_FILENAME);
            if not goals:
                sys.exit()
    except:
        rospy.logwarn('GoalMaker exiting: error reading parameter or file')
        sys.exit()


    rospy.init_node('GoalMaker')
    pub = rospy.Publisher('goal', Point)

    rospy.wait_for_service('getPos')
    getPos = rospy.ServiceProxy('getPos', GetPos)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            curPos = getPos().pos
            processPos(curPos)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        r.sleep()

    rospy.spin()
