#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy, sys, time
from std_msgs.msg import String
from threading import Thread

LEFT = 1
RIGHT = 2
select = 0
out = 3096

def output():
  global out
  global select
  pub = rospy.Publisher('psoc_cmd',String)
  while not rospy.is_shutdown():
    p = String()
    if select is LEFT:
      p.data = ">FLPC:"+str(out)
    elif select is RIGHT:
      p.data = ">FRPC:"+str(out)
    pub.publish(p)
    time.sleep(.1)

def raw_drive():
  global out
  global select
  rospy.init_node('psoc_raw_tank_drive')
  rospy.loginfo("Left or Right? Left = 1, Right = 2")
  t = Thread(target=output, args=[])
  t.start()
  select = int(raw_input())
  print "Enter Values, 'q' to quit"
  while not rospy.is_shutdown():
      inp = str(raw_input())
      if inp is 'q':
        rospy.signal_shutdown('user shutdown')
        sys.exit()
      else:
        out = int(inp)

if __name__ == "__main__":
  try:
    raw_drive()
  except rospy.ROSInterruptException:
    pass
