#!/usr/bin/env python
"""
  It generates state machines for serving sushi plate
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states

class SushiServingStateMachineGenerator:
  def generate(self):
    nav_states = states.NavigateTo()

    sm = smach.StateMachine(outcomes=['success','failure'])

    T = {'failure':'failure'}
    
    with sm:
      # Step 0 : Start State
      smach.StateMachine.add('start_state',states.Start(),transitions = {'success':'move_to_turntable'})

      # Step 1 : Move to turn table
      T['success'] = 'scan_table'
      T['failure'] = 'failure'
      smach.StateMachine.add('move_to_turntable',nav_states,transitions = T.copy())

      # Step 2 : Scan table
      smach.StateMachine.add('scan_table',states.ScanTable(),transitions = {'no_object':'success','objects':'success'})

      # Steps are missing. Fill in later

    return sm
