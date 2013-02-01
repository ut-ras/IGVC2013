#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy, sys, time
from geometry_msgs.msg import Twist
from threading import Thread
from math import pi

MAX_X_SPEED = float(2)
MAX_W_SPEED = pi
xOut = 0
wOut = 0
right_state = False

def twosCompInverse(inp):
  if inp is 0:
    return inp
  elif inp <=127:
    return -(inp+1)
  else:
    return -(inp - 255)

def output():
  global xOut
  global wOut
  global right_state
  pub = rospy.Publisher('vel_cmd',Twist)
  while not rospy.is_shutdown():
    if right_state:
      p = Twist()
      p.linear.x = xOut*MAX_X_SPEED/128 if right_state else 0
      p.angular.z = wOut*MAX_W_SPEED/128 if right_state else 0
      pub.publish(p)
    time.sleep(.1) #run at 10hz


def joystick():
  global xOut
  global wOut
  global right_state
  rospy.init_node('psoc_raw_polar_drive')
  rospy.loginfo("PSoC raw tank drive polar style using the logitech duel action is running")
  msgs = []
  t = Thread(target=output, args=[])
  t.start()
  with open("/dev/input/js0",'r') as pipe:
    for i in range(144): #there's ~144 bytes (18 packets of data) on initialization. dropping them
       pipe.read(1)
    while not rospy.is_shutdown():
      for char in pipe.read(1):
        msgs.append(ord(char))
        if len(msgs) is 8:
          if msgs[6] is 2 and msgs[7] is 0:
            wOut = twosCompInverse(msgs[5])
          elif msgs[6] is 2 and msgs[7] is 1:
            xOut = twosCompInverse(msgs[5])
          elif msgs[7] is 4:
            right_state = (msgs[4] is 1)
          msgs = []

if __name__ == "__main__":
  try:
    joystick()
  except rospy.ROSInterruptException:
    pass
