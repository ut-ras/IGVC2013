#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy, tty, sys, termios
from geometry_msgs.msg import Twist

LIN_INC = 1
ANG_INC = 1
pub_cmd = rospy.Publisher('vel_cmd', Twist)

if __name__ == "__main__":
    try:
      rospy.init_node('psoc_teleop_key')

      print("hit w/a/s/d to move, q to quit")

      flag = True

      while flag:
        # wait for key stroke
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
          tty.setraw(sys.stdin.fileno())
          ch = sys.stdin.read(1)
        finally:
          termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        p = Twist()
        p.linear.x = 0
        p.angular.z = 0

        if ch == "w" :
          print("forward!")
          p.linear.x = LIN_INC
        elif ch == "a" :
          print("clockwise!")
          p.linear.z = ANG_INC
        elif ch == "s" :
          print("backward!")
          p.angular.x = -LIN_INC
        elif ch == "d" :
          print("counter-clockwise!")
          p.linear.z = -ANG_INC
        elif ch == "q" :
          flag = False

        pub_cmd.publish(p)

    except rospy.ROSInterruptException: pass
