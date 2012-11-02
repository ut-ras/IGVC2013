#!/usr/bin/env python
"""
  It generates state machines for putdown objects
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states

class PutdownStateGenerator:
  def generate(self):
    sm = smach.StateMachine(outcomes=['success','failure'])
    
    T = {'failure':'failure'}
    
    with sm:
      T['success'] = 'success'
      T['failure'] = 'failure'
      smach.StateMachine.add('putdown',states.Putdown(),transitions = T.copy())

    return sm
