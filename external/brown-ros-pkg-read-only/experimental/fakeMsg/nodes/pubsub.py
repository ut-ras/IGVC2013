#!/usr/bin/env python
import roslib; roslib.load_manifest('fakeMsg')
import rospy
import random
from fakeMsg.msg import * 

receivedNum = -1

def process(msg):
  global receivedNum
  print "Received"
  print str(msg)
  receivedNum = msg.num


def fake():
  global receivedNum
  pub = rospy.Publisher('pubfake',pubMsg)

  rospy.init_node('fake')

  rospy.Subscriber('subfake',subMsg, process)
  num = 0

  while not rospy.is_shutdown():
    fakeTopic = pubMsg() 
    fakeTopic.str = "subfake"
    fakeTopic.num = num
    fakeTopic.lastReceivedNum = receivedNum
    
    pub.publish(fakeTopic)
    num = num + 1

    rospy.sleep(0.1)

if __name__ =="__main__":
  try:
    fake()
  except rospy.ROSInterruptException: pass
