#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')
import sys, rospy, os, math

from gazebo import gazebo_interface
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import *

#reference_frame = "torso_lift_link"
reference_frame = ""
f = open("/home/brownrobotics/toyblock.urdf",'r')
model_xml = f.read()
robot_namespace = rospy.get_namespace()

get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
physics = get_physics_properties()
paused = physics.pause
while (not paused):
	get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
	physics = get_physics_properties()
	paused = physics.pause
	if (not paused):
		rospy.wait_for_service('/gazebo/pause_physics')
		pause_client = rospy.ServiceProxy('gazebo/pause_physics', Empty)
		pause_client()

for block in range(0,42):
	column = int(math.floor( block / 7 ))
	row = block % 7
	initial_pose = Pose()
	initial_pose.position.x =  0.432 + 0.044 *  row
	initial_pose.position.y = -0.40  - 0.1   *  math.floor( column / 3 )
	initial_pose.position.z =  0.443 + 0.045 * (column % 3 )
	strColumn = str(column)
	strRow = str(row)
	model_name = 'block' + strColumn + strRow
	
	print("Adding block" + strColumn + strRow)
	rospy.wait_for_service('/gazebo/spawn_urdf_model')
	try:
		spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModel)
		resp = spawn_urdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
		print("spawn status: ",resp.status_message)
		#return resp.success
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

rospy.wait_for_service('/gazebo/unpause_physics')
pause_client = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
pause_client()