#!/usr/bin/env python
"""
  It generates state machines for setting table
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states,PickupStateGenerator,PutdownStateGenerator

objects_to_pickup = ['plate','bowl','cup','chopsticks']

class TableSettingStateMachineGenerator:
  def generate(self):
    pickup_sm = PickupStateGenerator.PickupStateGenerator().generate()    
    putdown_sm = PutdownStateGenerator.PutdownStateGenerator().generate()    
    nav_states = states.NavigateTo()

    sm = smach.StateMachine(outcomes=['success','failure'])

    T = {'failure':'failure'}
    
    with sm:
      # Step 0 : Start State
      smach.StateMachine.add('start_state',states.Start(),transitions = {'success':'move_to_shelf'})

      # Step 1 : Move to shelf
      T['success'] = 'scan_table'
      T['failure'] = 'failure'
      smach.StateMachine.add('move_to_shelf',nav_states,transitions = T.copy())

      # Step 2 : Scan shelf
      smach.StateMachine.add('scan_table',states.ScanTable(),transitions = {'no_object':'success','objects':'pickup'})

      # Step 3 : Pickup specific objects
      T['success'] = 'move_to_servtable'
      T['failure'] = 'failure'
      smach.StateMachine.add('pickup',pickup_sm,transitions = T.copy())

      # Step 4 : Move to serv Table
      T['success'] = 'putdown'
      T['failure'] = 'failure'
      smach.StateMachine.add('move_to_servtable',nav_states,transitions = T.copy())

      # Step 4 : Putdown
      T['success'] = 'check_status' 
      T['failure'] = 'failure'
      smach.StateMachine.add('putdown',putdown_sm,transitions = T.copy())

      # Step 5 : check if all objects are placed
      T['success'] = 'success'
      T['failure'] = 'move_to_shelf'
      smach.StateMachine.add('check_status',states.CheckStatus(), transitions = T.copy())

    return sm
