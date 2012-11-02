#!/usr/bin/env python
"""
  It generates state machines for celebration
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states

class CelebrateSuccessStateMachineGenerator:
  def generate(self):
    sm = smach.StateMachine(outcomes=['success'])

    T = {'success':'failure'}
    
    with sm:
      # Step 0 : Start State
      T['success'] = 'celebrate'
      smach.StateMachine.add('start_state',states.Start(),transitions = T.copy())


      T['success'] = 'success' 
      smach.StateMachine.add('celebrate',states.Celebrate(),transitions = T.copy())

    return sm

