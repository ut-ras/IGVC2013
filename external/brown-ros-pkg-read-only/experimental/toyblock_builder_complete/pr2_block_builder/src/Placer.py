#! /usr/bin/env python
import roslib; roslib.load_manifest('pr2_block_builder')
import rospy
import actionlib

import pr2_block_builder_msgs.msg 

class Placer():

  aborted = False

  def __init__(self):
    # creates the action client, passing place command action
    self.placerClient = actionlib.SimpleActionClient('/pr2_block_builder/place',pr2_block_builder_msgs.msg.PlaceCommandAction)

    rospy.loginfo("Wait for Server")
    # wait until server is up
    self.placerClient.wait_for_server()


      
  # Invoke whatever placing necessary.  Method should block until action completed.  
  # Returns "completed", "failed" or "aborted"
  def place(self, block):
    goal = pr2_block_builder_msgs.msg.PlaceCommandGoal()
  
    x = block.position.x;
    y = block.position.y;
    z = block.position.z;

    goal.x = x
    goal.y = y
    goal.z = z

    print str(goal)
    
    rospy.loginfo("Sending Goal..")
    self.placerClient.send_goal(goal)
    self.aborted = False

    self.placerClient.wait_for_result()

    if self.placerClient.get_result().result:
        return "completed"
    else:
        return "aborted"

  # Cancels the place motion.  Robot should freeze.  
  # This method should do nothing if the robot is not currently placing.
  def cancel(self):
    self.placerClient.cancel_goal()  
    self.aborted = True

