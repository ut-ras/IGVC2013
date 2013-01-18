#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy, sys, time
from std_msgs.msg import String
from threading import Thread

leftOut = 0
rightOut = 0

def twosCompInverse(inp):
  if inp is 0:
    return inp
  elif inp <=127:
    return -(inp+1)
  else:
    return -(inp - 255)

def output():
  global leftOut
  global rightOut
  pub = rospy.Publisher('psoc_cmd',String)
  while not rospy.is_shutdown():
    p = String()
    p.data = ">SPLM:"+str(leftOut)
    pub.publish(p)
    p = String()
    p.data = ">SPRM:"+str(rightOut)
    pub.publish(p)
    time.sleep(.1)


def joystick():
  global leftOut
  global rightOut
  rospy.init_node('psoc_raw_tank_drive')
  rospy.loginfo("PSoC raw tank drive using the logitech duel action is running")
  msgs = []
  t = Thread(target=output, args=[])
  t.start()
  with open("/dev/input/js0",'r') as pipe:
    for i in range(144): #there's 144 bytes (18 packets of data) on initialization. dropping them
       pipe.read(1)
    while not rospy.is_shutdown():
      for char in pipe.read(1):
        msgs.append(ord(char))
        if len(msgs) is 8:
          if msgs[6] is 2 and msgs[7] is 1:
            leftOut = twosCompInverse(msgs[5])
          elif msgs[6] is 2 and msgs[7] is 3:
            rightOut = twosCompInverse(msgs[5])
          msgs = []

if __name__ == "__main__":
  try:
    joystick()
  except rospy.ROSInterruptException:
    pass
