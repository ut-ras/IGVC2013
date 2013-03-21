#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy

from ReactiveUtils import ReactiveUtils

from filters.msg import EKFData
from geometry_msgs.msg import Point

goals = [Point(0, 0, 0), Point(8, 0, 0), Point(0, 0, 0)]
curGoalIndex = 0

pub = None

def ekf_callback(data):
    global curGoalIndex

    if curGoalIndex < len(goals):
        pos = Point(data.x_pos, data.y_pos, 0)

        distToGoal = ReactiveUtils.euclidDistPoint(pos, goals[curGoalIndex])

        if distToGoal < ReactiveUtils.CLOSE_ENOUGH_TO_GOAL:
            print "close enough to the goal, moving to the next waypoint"
            curGoalIndex += 1

        if curGoalIndex < len(goals):
            pub.publish(goals[curGoalIndex])

if __name__ == "__main__":
    rospy.init_node('GoalMaker')
    sub = rospy.Subscriber("ekf_data", EKFData, ekf_callback)
    pub = rospy.Publisher('goal', Point)
    rospy.spin()
