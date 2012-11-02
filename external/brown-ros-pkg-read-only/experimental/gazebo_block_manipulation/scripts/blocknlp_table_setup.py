#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo_block_manipulation')
import sys, rospy, os, math
from block_build_msgs.msg import *

from gazebo import gazebo_interface
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from std_srvs.srv import *
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

TORSO_TOPIC_NAME = "/torso_controller/state"

def spawn_models():
    reference_frame = ""
    pkg_dir = roslib.packages.get_pkg_dir('gazebo_block_manipulation')
    f_red = open(pkg_dir + "/objects/redblock.urdf",'r')
    f_blue = open(pkg_dir + "/objects/blueblock.urdf",'r')
    f_table = open(pkg_dir + "/objects/table.urdf", 'r')
    red_model_xml = f_red.read()
    blue_model_xml = f_blue.read()
    table_model_xml = f_table.read()
    robot_namespace = rospy.get_namespace()
    get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics = get_physics_properties()
    paused = physics.pause

    while not paused:
            get_physics_properties = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
            physics = get_physics_properties()
            paused = physics.pause
            if (not paused):
                    rospy.wait_for_service('/gazebo/pause_physics')
                    pause_client = rospy.ServiceProxy('gazebo/pause_physics', Empty)
                    pause_client()

    initial_pose = Pose()
    initial_pose.position.x =  0.6
    initial_pose.position.y =  -0.1
    initial_pose.position.z =  0.72
    table_pose = Pose()
    table_pose.position.x =  0.65
    table_pose.position.y =  0
    table_pose.position.z =  0.0
    table_pose.orientation.x = 0
    table_pose.orientation.y = 0
    table_pose.orientation.z = 0.707
    table_pose.orientation.w = 0.707

    red_model_name = 'redblock'
    blue_model_name = 'blueblock'
    table_model_name = 'table'

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
            spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModel)
            resp = spawn_urdf_model(table_model_name, table_model_xml, robot_namespace, table_pose, reference_frame)
            print("spawn status: ",resp.status_message)
            #return resp.success
    except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
            spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModel)
            resp = spawn_urdf_model(red_model_name, red_model_xml, robot_namespace, initial_pose, reference_frame)
            print("spawn status: ",resp.status_message)
            #return resp.success
    except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    initial_pose.position.y = 0.1
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
            spawn_urdf_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModel)
            resp = spawn_urdf_model(blue_model_name, blue_model_xml, robot_namespace, initial_pose, reference_frame)
            print("spawn status: ",resp.status_message)
            #return resp.success
    except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    rospy.wait_for_service('/gazebo/unpause_physics')
    pause_client = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
    pause_client()
    rospy.signal_shutdown("Spawned all models")

def torsoCallback(state):
    if state.actual.positions[0] > 0.2:
        spawn_models()

if __name__ == '__main__':
    rospy.init_node('block_build_manager')

    torso_command_listner = rospy.Subscriber(TORSO_TOPIC_NAME, JointTrajectoryControllerState, torsoCallback)
    rospy.spin()
