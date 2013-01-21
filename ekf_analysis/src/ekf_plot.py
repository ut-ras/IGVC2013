#!/usr/bin/env python

import roslib; roslib.load_manifest('ekf_analysis')
import rospy
from filters.msg import EKFData
from Tkinter import *

def update_canvas(ekfData):
    rospy.loginfo(rospy.get_name() + ": Entered Callback. Will try tp print canvas")

    root = Tk()
    w = Label(root, text='Hello world!!')
    w.pack()

    root.mainloop()

def listener():
    rospy.init_node('ekf_analysis', anonymous=False)
    rospy.Subscriber('ekf_data', EKFData, update_canvas)
    rospy.spin()

if __name__ == '__main__':
    listener()
