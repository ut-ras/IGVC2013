#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_block_builder')
import rospy
from Builder import *
from pr2_block_builder_msgs.msg import *
from pr2_block_builder_msgs.srv import *

"""
Global Variables
"""
builder = None

# Initialises the ROS node
def startup():
    rospy.init_node('pr2_block_builder')

    # Register the services being provided
    rospy.Service('pr2_block_builder/Start', Start, start)
    rospy.Service('pr2_block_builder/Stop', Stop, stop)
    rospy.Service('pr2_block_builder/Restart', Restart, restart)

    # Create the builder
    global builder
    builder = Builder()

# Receives castle structure from front-end
def start(req): 
    rospy.loginfo("Starting...")

    global builder
    blocks = req.blocks.blocks
    builder.start(blocks)

    return StartResponse()

# Front-end requested to stop buliding
def stop(req): 
    rospy.loginfo("Stopping...")

    global builder
    builder.stop()

    return StopResponse()

def restart(req): 
    rospy.loginfo("Restart signalled.")

    global builder
    builder.restart()

    return RestartResponse()

if __name__ == '__main__':
     try:
         startup()
         rospy.spin()
     except rospy.ROSInterruptException: pass
