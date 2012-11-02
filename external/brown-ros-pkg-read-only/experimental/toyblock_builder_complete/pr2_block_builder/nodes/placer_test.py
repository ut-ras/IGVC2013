#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_block_builder')
import rospy

from Placer import *
from Picker import *
from pr2_block_builder_msgs.msg import *
from pr2_block_builder_msgs.srv import *
from geometry_msgs.msg import *




def pp(x,y,z,num):
  global picker
  global p

  pose = Pose()
  pose.position.x = x
  pose.position.y = y
  pose.position.z = z
  picker.pick(num)
  rospy.loginfo(p.place(pose))

if __name__ == '__main__':
  rospy.init_node('placer_test')

  global picker
  global p
  picker = Picker()
  p = Placer()

  pp(0,0,0,0)
  """
  pp(0,1,0,1)
  pp(0,2,0,2)
  pp(1,2,0,3)
  pp(2,2,0,4)
  pp(0,1,0,5)
  pp(0,2,0,6)
  pp(0,0,1,7)
  """









