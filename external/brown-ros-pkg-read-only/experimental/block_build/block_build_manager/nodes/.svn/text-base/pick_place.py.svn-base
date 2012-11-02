#!/usr/bin/env python
import roslib; roslib.load_manifest('block_build_manager')
import rospy
import actionlib
import sys

from block_build_msgs.msg import *
from geometry_msgs.msg import Quaternion

PICK_COMMAND_NAME = "/blocknlp/command"
PLACE_COMMAND_NAME = "/blocknlp/command"

def sendpickgoal(x):
  global pick_client
  goal = block_build_msgs.msg.CommandGoal()

  goal_command = 0
  goal.obj_num = x
  rospy.loginfo("sending Goal")
  pick_client.send_goal(goal)

def sendplacegoal(x, y):
  global placeclient
  goal = block_build_msgs.msg.CommandGoal()

  goal.command = 1
  goal.x = x
  goal.y = y
  #goal.z = 0

  #orientation = Quaternion(0, 0.707, 0, 0.707)
  #goal.orientation = orientation
  #goal.precise_place = True

  print str(goal)
  rospy.loginfo("Sending Goal..")
  place_client.send_goal(goal)
  #self.aborted = False

  place_client.wait_for_result()

  if place_client.get_result().result:
      rospy.loginfo("Complete")
  else:
      rospy.loginfo("Fail")


if __name__ == '__main__':
  global pick_client, place_client

  rospy.init_node('test_node')

  # initialize action client
  pick_client = actionlib.SimpleActionClient(PICK_COMMAND_NAME,block_build_msgs.msg.CommandAction)
  rospy.loginfo("Wait for server pick server")
  pick_client.wait_for_server()

  place_client = actionlib.SimpleActionClient(PLACE_COMMAND_NAME,block_build_msgs.msg.CommandAction)
  rospy.loginfo("Wait for server place server")
  place_client.wait_for_server()

  sendpickgoal(int(sys.argv[1]))
  pick_client.wait_for_result()

  if pick_client.get_result().result:
    rospy.loginfo("Pick up block success")
    sendplacegoal(float(sys.argv[2]), float(sys.argv[3]))
    place_client.wait_for_result()
    if place_client.get_result().result:
      rospy.loginfo("Place block success")
  else:
    rospy.loginfo("Fail") 





