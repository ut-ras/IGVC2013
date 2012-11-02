#!/usr/bin/env python
import roslib; roslib.load_manifest('block_build_manager')
import rospy
import actionlib
import sys

from block_build_msgs.msg import *

def sendgoal(x):
  global aclient
  goal = block_build_msgs.msg.CommandGoal()

  goal.command = 0
  goal.obj_num = x
  rospy.loginfo("sending Goal")
  aclient.send_goal(goal)



if __name__ == '__main__':
  global aclient

  rospy.init_node('test_node')

  # initialize action client
  aclient = actionlib.SimpleActionClient('/blocknlp/command',block_build_msgs.msg.CommandAction)
  rospy.loginfo("Wait for server command server")
  aclient.wait_for_server()

  sendgoal(int(sys.argv[1]))

  aclient.wait_for_result()
  if aclient.get_result().result:
    rospy.loginfo("Complete")
  else:
    rospy.loginfo("Fail")



