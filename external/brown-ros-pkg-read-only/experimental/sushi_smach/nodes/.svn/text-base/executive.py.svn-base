#!/usr/bin/env python
"""
  Author : Jihoon Lee <jihoonlee.in@gmail.com>
  Date   : May 2012

  Sushi Executive

  Some code are adopted from Lorenzo Riano's sushi_sm package from PR2 workshop
"""
import roslib; roslib.load_manifest("sushi_smach")
import rospy
import smach
import smach_ros
from sushi_smach import states,TableCleanStateMachineGenerator,TableSettingStateMachineGenerator,SushiServingStateMachineGenerator,CelebrateSuccessStateMachineGenerator

class SushiExecutive:

  sm = None

  def __init__(self):
    table_clean_sm = TableCleanStateMachineGenerator.TableCleanStateMachineGenerator().generate()
    table_setting_sm = TableSettingStateMachineGenerator.TableSettingStateMachineGenerator().generate()
    sushi_serving_sm = SushiServingStateMachineGenerator.SushiServingStateMachineGenerator().generate()
    celebrate_sm = CelebrateSuccessStateMachineGenerator.CelebrateSuccessStateMachineGenerator().generate()

    sm = smach.StateMachine(outcomes=['final_success','final_failure'])

    rospy.loginfo("Creating State Machine")
  
    T = {'failure':'failure'}
    nav_states = states.NavigateTo()

    with sm:
      # Step 0 : Start State
      smach.StateMachine.add('start_state',states.Start(),transitions = {'success':'clean_table'})

      # Step 1 : Table Cleaning
      T['success'] = 'set_table'
      T['failure'] = 'final_failure'
      smach.StateMachine.add('clean_table',table_clean_sm,transitions=T.copy())

      # Step 3 : Table Setting
      T['success'] = 'serv_sushi'
      T['failure'] = 'final_failure'
      smach.StateMachine.add('set_table',table_setting_sm,transitions=T.copy())
      
      # Step 4 : Serv sushi plates
      T['success'] = 'celebrate_it'
      T['failure'] = 'final_failure' 
      smach.StateMachine.add('serv_sushi',sushi_serving_sm,transitions=T.copy())

      # Step 5 : Celebrates a success
      smach.StateMachine.add('celebrate_it',celebrate_sm,transitions={'success':'final_success'})

    self.sm = sm

  def spin(self):
    sis = smach_ros.IntrospectionServer('server',self.sm,'/sm_root')
    sis.start()
    rospy.loginfo("Executing State Machine...")
    outcome = self.sm.execute()
    rospy.loginfo("outcome = " + str(outcome))
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
  try:
    rospy.init_node("sushi_executive")
    executive = SushiExecutive()
    rospy.loginfo("Initialized")
    executive.spin()
  except rospy.ROSInterruptException: pass



