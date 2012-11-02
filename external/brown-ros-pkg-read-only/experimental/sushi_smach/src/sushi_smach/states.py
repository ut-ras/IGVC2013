
"""
  States
"""

import roslib; roslib.load_manifest('sushi_smach')
import rospy
import smach
import smach_ros

class Start(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success'])

  def execute(self, userdata):
    rospy.loginfo('Executing Start State')

    return 'success'

class ScanTable(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['objects','no_object'])
    self.counter = 1

  def execute(self, userdata):
    rospy.loginfo('Executing Scan Table')

    if self.counter == 1:
      self.counter = 2
      return 'objects'
    else:
      return 'no_object'
    
class NavigateTo(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success','failure'])

  def execute(self, userdata):
    rospy.loginfo('Executing Navigate To')

    return 'success'

class Celebrate(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success'])

  def execute(self, userdata):
    rospy.loginfo('Executing Celebration')

    return 'success'

class Pick(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success','failure'])

  def execute(self, userdata):
    rospy.loginfo('Executing Pick')

    return 'success'

class Putdown(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success','failure'])

  def execute(self, userdata):
    rospy.loginfo('Executing Putdown')

    return 'success'

class MovePutdown(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success','failure'])

  def execute(self, userdata):
    rospy.loginfo('Executing Move Putdown')

    return 'success'

class CheckStatus(smach.State): 
  def __init__(self):
    smach.State.__init__(self,outcomes=['success','failure'])

  def execute(self, userdata):
    rospy.loginfo('Executing Check Status')

    return 'success'

