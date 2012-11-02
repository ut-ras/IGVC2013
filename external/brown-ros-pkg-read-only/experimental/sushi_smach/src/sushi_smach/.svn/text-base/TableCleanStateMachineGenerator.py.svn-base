#!/usr/bin/env python
"""
  It generates state machines for cleaning table
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states,PickupStateGenerator

class TableCleanStateMachineGenerator:

  def generate(self):

    pickup_sm = PickupStateGenerator.PickupStateGenerator().generate()    
    nav_states = states.NavigateTo()

    sm = smach.StateMachine(outcomes=['success','failure'])

    T = {'failure':'failure'}
    
    with sm:
      # Step 0 : Start State
      smach.StateMachine.add('start_state',states.Start(),transitions = {'success':'move_to_servtable'})

      # Step 1 : Move to serving table
      T['success'] = 'scan_table'
      T['failure'] = 'failure'
      smach.StateMachine.add('move_to_servtable',nav_states,transitions = T.copy())

      # Step 2 : Scan Table and update World State
      smach.StateMachine.add('scan_table',states.ScanTable(),transitions = {'no_object':'success','objects':'pickup'})

      # Step 3 : Pick up objects
      T['success'] = 'move_putdown'
      T['failure'] = 'failure'
      smach.StateMachine.add('pickup',pickup_sm,transitions = T.copy())
      
      # Step 4 : Move and Put down
      T['success'] = 'move_to_servtable' 
      T['failure'] = 'failure'
      smach.StateMachine.add('move_putdown',states.MovePutdown(),transitions = T.copy())

    return sm
