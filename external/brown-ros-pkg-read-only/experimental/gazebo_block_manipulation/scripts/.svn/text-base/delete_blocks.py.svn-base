#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')
import sys, rospy, os, math

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

for block in range(0,42):
	column = int(math.floor( block / 7 ))
	row = block % 7
	strColumn = str(column)
	strRow = str(row)
	model_name = 'block' + strColumn + strRow	
	print("Deleting block" + strColumn + strRow)
	rospy.wait_for_service('/gazebo/delete_model')
	try:
		delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		reps = delete_model(model_name)
		print("delete status: " + reps.status_message)
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)