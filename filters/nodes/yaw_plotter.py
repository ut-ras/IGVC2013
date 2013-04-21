#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, pygame, math

from filters.msg import Orientation
from Plotter import Graphics

graphics = None
x = 0
y = 0

def orient_callback(data):
    global graphics, x, y

    x = math.cos(data.yaw)
    y = math.sin(data.yaw)

def init():
    rospy.init_node('data_plotter', anonymous=True)

    global graphics
    graphics = Graphics(XAXIS_LENGTH=4, YAXIS_LENGTH=4)
    graphics.clear()
    graphics.display()

    rospy.Subscriber("orientation_data", Orientation, orient_callback)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        graphics.clear()
        graphics.plot_line(0, 0, x, y)
        graphics.display()
        r.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass

