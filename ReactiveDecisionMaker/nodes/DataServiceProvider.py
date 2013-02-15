#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')

import rospy

from ReactiveDecisionMaker.srv import *
from filters.msg import EKFData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

x_pos = 0
y_pos = 0
heading = None
scan = None
goal = Point(10, 0, 0)

def handle_getXPos(req):
    return GetXPosResponse(y_pos)

def handle_getYPos(req):
    return GetYPosResponse(x_pos)

def handle_getHeading(req):
    return GetHeadingResponse(heading)

def handle_getScan(req):
    return GetScanResponse(scan)

def handle_getGoal(req):
    return GetGoalResponse(goal)

def init_server():
    rospy.init_node('DataServiceProvider')
    serv1 = rospy.Service('getXPos', GetXPos, handle_getXPos)
    serv2 = rospy.Service('getYPos', GetYPos, handle_getYPos)
    serv3 = rospy.Service('getHeading', GetHeading, handle_getHeading)
    serv4 = rospy.Service('getScan', GetScan, handle_getScan)
    serv5 = rospy.Service('getGoal', GetGoal, handle_getGoal)

def ekf_callback(data):
    global x_pos, y_pos, heading
    x_pos = data.x_pos
    y_pos = data.y_pos
    heading = data.yaw

def scan_callback(data):
    global scan
    scan = data

def goal_callback(data):
    global goal
    goal = data

def init_subscriptions():
    sub1 = rospy.Subscriber("ekf_data", EKFData, ekf_callback)
    sub2 = rospy.Subscriber("scan", LaserScan, scan_callback)
    sub3 = rospy.Subscriber("goal", Point, goal_callback)

if __name__ == "__main__":
    init_server()
    init_subscriptions()
    rospy.spin()
