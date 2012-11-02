#! /usr/bin/env python
import roslib; roslib.load_manifest('pr2_block_builder')
import rospy
import actionlib

import pr2_block_builder_msgs.msg 

class Picker():

    aborted = False

    def __init__(self):
    # creates the action client, passing place command action
      self.pickerClient = actionlib.SimpleActionClient('/pr2_block_builder/pick',pr2_block_builder_msgs.msg.PickCommandAction)
      rospy.loginfo("Wait for Server")
    # wait until server is up
      self.pickerClient.wait_for_server()

    # Invoke whatever picking necessary.  Returns "completed", "failed" or "aborted"
    def pick(self, blockNumber):
        goal = pr2_block_builder_msgs.msg.PickCommandGoal(number=blockNumber)
        rospy.loginfo("Sending Goal..")
        self.pickerClient.send_goal(goal)
        self.aborted = False

	self.pickerClient.wait_for_result()

        if self.pickerClient.get_result().result:
            return "completed"
        else:
            return "aborted"

    # Cancels the pick motion.  Robot should freeze.  This method should do nothing if 
    # the robot is not currently picking
    def cancel(self):
        self.pickerClient.cancel_goal()
        self.aborted = True
