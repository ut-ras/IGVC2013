#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from std_msgs.msg import String
import Tkinter

pub = rospy.Publisher('psoc_cmd', String)

def data_callback(data):
    pass

def ctrl_pan():
    rospy.init_node('PSoC_Listener')
    sub = rospy.Subscriber('psoc_data', PSoC, data_callback)

root = Tkinter.Tk('test')
Tkinter.ttk.Button(root,text="Test").grid()
root.mainloop()


if __name__ is '__main__':
    try:
        ctrl_pan()
    except rospy.ROSInterruptException: pass
