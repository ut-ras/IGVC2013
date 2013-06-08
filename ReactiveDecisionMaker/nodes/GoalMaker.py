#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from ReactiveUtils import *

from ReactiveDecisionMaker.srv import *
from geometry_msgs.msg import Point

WAYPOINT_FILENAME = '/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/offsets'

goals = [
    Point(20, 0, 0),
    Point(0, 0, 0)
    ]

curGoalIndex = 0

pub = None

def processPos(pos):
    global curGoalIndex

    if curGoalIndex < len(goals):
        pos = Point(pos.x, pos.y, 0)

        distToGoal = euclidDistPoint(pos, goals[curGoalIndex])

        if distToGoal < CLOSE_ENOUGH_TO_GOAL:
            print "close enough to ", goals[curGoalIndex].x, ",", goals[curGoalIndex].y
            curGoalIndex += 1
            if curGoalIndex < len(goals):
                print "setting new goal to ", goals[curGoalIndex].x, ",", goals[curGoalIndex].y

        if curGoalIndex < len(goals):
            print "publishing goal ", goals[curGoalIndex]
            pub.publish(goals[curGoalIndex])

def readWaypoints(filename):
    print 'reading waypoints'

    try:
        f = open(filename, 'r')
    except:
        print 'no file found named ' + filename
        return

    list = []

    for line in f:
        s = line.split()

        try:
            x = float(s[0])
            y = float(s[1])
        except:
            print 'error converting to floats: ' + line
            return

        list.append(Point(x, y, 0));

    return list

if __name__ == "__main__":
    try:
        if bool(rospy.get_param('GoalMaker/usefile', False)):
            goals = readWaypoints(WAYPOINT_FILENAME);
    except: pass

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
