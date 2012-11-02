#!/usr/bin/env python
import roslib; roslib.load_manifest('block_build_manager')
import rospy
import actionlib
import sys

from block_build_msgs.msg import *
from geometry_msgs.msg import Quaternion

PLACE_COMMAND_NAME = "/blocknlp/command"

def sendplacegoal(x, y):
  global place_client
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
  global place_client

  rospy.init_node('test_node')

  place_client = actionlib.SimpleActionClient(PLACE_COMMAND_NAME,block_build_msgs.msg.CommandAction)
  rospy.loginfo("Wait for command server")

  sendplacegoal(float(sys.argv[1]), float(sys.argv[2]))
  place_client.wait_for_result()
  if place_client.get_result().result:
    rospy.loginfo("Place block success")
  else:
    rospy.loginfo("Fail") 





