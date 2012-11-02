#!/usr/bin/env python

import roslib; roslib.load_manifest('fakeMsg')
import rospy
import struct
import random
import json
import sys

from fakeMsg.msg import * 

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_f = struct.Struct("<f")

# It converts byte array string to int list
def __convertUINTARRAY(inst):
  uintList = []
    
  i = 0
  while i < len(inst):
    (val, ) = _struct_B.unpack(inst[i:i+1])
    uintList.append(val)
    i = i + 1
  return uintList

def process(msg):
  print "------- Entire Msg -------------"
  print msg
  print "--------------------------------"

  print "------- msg.unsigned_data ------"
  print str(msg.unsigned_data)
  print "--------------------------------"
  print "Type of unsigned_data = " + str(type(msg.unsigned_data)) +"\n"

  print "------- msg.unsigned_data unpacked ----"
  print str(__convertUINTARRAY(msg.unsigned_data))
  print "---------------------------------------"

def test():
  sub = rospy.Subscriber('fake',FakeMsg, process)
  rospy.init_node('test')

  rospy.spin()

if __name__ == "__main__":
  try:
    test()
  except rospy.ROSInterruptException: pass
