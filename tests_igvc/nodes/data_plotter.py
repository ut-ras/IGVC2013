#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, pygame

from vn_200_imu.msg import vn_200_accel_gyro_compass
from Plotter import Graphics

graphics = None

totalx = 0
totaly = 0
counter = 0.0

X_OFFSET = 0.0
Y_OFFSET = 0.0

def vn_200_imu_callback(data):
    global graphics, X_OFFSET, Y_OFFSET
    graphics.plot(data.compass.x - X_OFFSET, data.compass.y - Y_OFFSET)
    graphics.display()

    global totalx, totaly, counter
    totalx += data.compass.x
    totaly += data.compass.y
    counter += 1.0
    print totalx/counter, totaly/counter

def init():
    rospy.init_node('data_plotter', anonymous=True)

    global graphics
    graphics = Graphics(XAXIS_LENGTH=8, YAXIS_LENGTH=8)
    graphics.clear()
    graphics.display()

    rospy.Subscriber("vn_200_accel_gyro_compass", vn_200_accel_gyro_compass, vn_200_imu_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass

