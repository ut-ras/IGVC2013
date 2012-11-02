#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_block_builder')
import rospy, math	

from Placer import *
from Picker import *
from pr2_block_builder_msgs.msg import *
from pr2_block_builder_msgs.srv import *
from geometry_msgs.msg import *

def reset():
  for block in range(0,42):
    layer  = int(math.floor( block / 7 ) / 2)
    column = int(math.floor( block / 7 ) % 2)
    row = block % 7
    initial_pose = Pose()
    initial_pose.position.x =  7 - row
    initial_pose.position.y =  -9 - column
    initial_pose.position.z =  layer
    initial_pose.orientation.x = 0
    initial_pose.orientation.y = 0.707
    initial_pose.orientation.z = 0
    initial_pose.orientation.w = 0.707
    strColumn = str( column )
    strRow    = str( row )
    picker.pick(-1)
    rospy.loginfo(p.place(initial_pose, True))

if __name__ == '__main__':
  rospy.init_node('reset_blocks')

  global picker
  global p
  picker = Picker()
  p = Placer()
  reset()











