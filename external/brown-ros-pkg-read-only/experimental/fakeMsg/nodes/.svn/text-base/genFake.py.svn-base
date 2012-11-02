#!/usr/bin/env python
import roslib; roslib.load_manifest('fakeMsg')
import rospy
import random
from fakeMsg.msg import * 

def genData():
  data = [115, 198,138,190]
  """,
          158,62,85,62,
          251,133,243,62,
          0,0,0,0]
  ,
          173,113,243,62,
          9,187,85,62,
          191,194,243,62,
          0,0,0,0,
          248,249,138,190,
          199,138,89,62,
          121,187,241,62,
         0,0,0,0]
  """
  return data

def fake():
  pub = rospy.Publisher('fake',FakeMsg)

  rospy.init_node('fake')

  while not rospy.is_shutdown():
    fakeTopic = FakeMsg() 
  
    fakeTopic.unsigned = genData()
    
    pub.publish(fakeTopic)

    rospy.sleep(4)




if __name__ =="__main__":
  try:
    fake()
  except rospy.ROSInterruptException: pass


