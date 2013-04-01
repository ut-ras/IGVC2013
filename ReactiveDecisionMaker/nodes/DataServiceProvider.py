#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')

import rospy

from ReactiveDecisionMaker.srv import *
from filters.msg import EKFData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

pos = Point(0, 0, 0)
heading = 0
scan = None
goal = Point(0, 0, 0)

def handle_getPos(req):
    return GetPosResponse(pos)

def handle_getHeading(req):
    return GetHeadingResponse(heading)

def handle_getScan(req):
    return GetScanResponse(scan)

def handle_getGoal(req):
    return GetGoalResponse(goal)

def init_server():
    rospy.init_node('DataServiceProvider')
    serv1 = rospy.Service('getPos', GetPos, handle_getPos)
    serv2 = rospy.Service('getHeading', GetHeading, handle_getHeading)
    serv3 = rospy.Service('getScan', GetScan, handle_getScan)
    serv4 = rospy.Service('getGoal', GetGoal, handle_getGoal)

def ekf_callback(data):
    global pos, heading
    pos.x = data.x_pos
    pos.y = data.y_pos
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
