#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')

import rospy

from ReactiveUtils import *
from ReactiveDecisionMaker.srv import *
from ReactiveDecisionMaker.msg import PlanarData
from filters.msg import EKFData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist

GOAL_TIMEOUT = .5

pos = Point(0, 0, 0)
heading = 0
pdata = None
goal = Point(0, 0, 0)
twist = Twist()

def handle_getPos(req):
    return GetPosResponse(pos)

def handle_getHeading(req):
    return GetHeadingResponse(heading)

def handle_getPlanarData(req):
    return GetPlanarDataResponse(pdata)

latestGoalTime = None

def handle_getGoal(req):
    curTime = rospy.get_time()

    if curTime - latestGoalTime < GOAL_TIMEOUT:
        return GetGoalResponse(goal)
    else:
        rospy.loginfo("goal data is too old!")
        return GetGoalResponse(Point(0, 0, TIMEOUT_ERROR))
    return

def handle_getTwist(req):
    return GetTwistResponse(twist)


def init_server():
    rospy.init_node('DataServiceProvider')
    serv1 = rospy.Service('getPos', GetPos, handle_getPos)
    serv2 = rospy.Service('getHeading', GetHeading, handle_getHeading)
    serv3 = rospy.Service('getPlanarData', GetPlanarData, handle_getPlanarData)
    serv4 = rospy.Service('getGoal', GetGoal, handle_getGoal)
    serv5 = rospy.Service('getTwist', GetTwist, handle_getTwist)

def ekf_callback(data):
    global pos, heading
    pos.x = data.x_pos
    pos.y = data.y_pos
    heading = data.yaw
    twist.linear.x = data.linear_velocity
    twist.angular.z = data.yaw_rate

def pdata_callback(data):
    global pdata
    pdata = data

def goal_callback(data):
    global goal, latestGoalTime
    goal = data
    latestGoalTime = rospy.get_time()

def init_subscriptions():
    sub1 = rospy.Subscriber("ekf_data", EKFData, ekf_callback)
    sub2 = rospy.Subscriber("planar_data", PlanarData, pdata_callback)
    sub3 = rospy.Subscriber("goal", Point, goal_callback)

if __name__ == "__main__":
    init_server()
    init_subscriptions()
    rospy.loginfo("server is up & we've subscribed to topics!");
    rospy.spin()
