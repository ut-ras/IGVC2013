#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, pygame, math

from sensor_msgs.msg import LaserScan
from ReactiveDecisionMaker.msg import PlanarData
from Plotter import Graphics

graphics = None

def plot_scan(ranges, angles, color):
    global graphics

    for i in range(len(ranges)):
        r = ranges[i]
        angle = angles[i]
        x = r*math.cos(angle)
        y = r*math.sin(angle)
        graphics.plot_line(0, 0, x, y, color)

        if i != len(ranges) - 1:
            r2 = ranges[i + 1]
            angle2 = angles[i + 1]
            x2 = r2*math.cos(angle2)
            y2 = r2*math.sin(angle2)
            graphics.plot_line(x, y, x2, y2, color)

ranges = [[], [], [], []]
angles = [[], [], [], []]
colors = [(0,0,255), (0,255,255), (0,255,0), (255,0,0)]

def draw_stuff():
    global graphics, ranges, angles
    graphics.clear()

    for i in range(len(ranges)):
        plot_scan(ranges[i], angles[i], colors[i])

    graphics.display()

def sonar_callback(data):
    global ranges, angles
    ranges[0] = data.ranges
    angles[0] = [data.angle_min + data.angle_increment*i for i in range(len(data.ranges))]

def sonar_scan_callback(data):
    global ranges, angles
    ranges[1] = data.ranges
    angles[1] = [data.angle_min + data.angle_increment*i for i in range(len(data.ranges))]

def image_scan_callback(data):
    global ranges, angles
    ranges[2] = data.ranges
    angles[2] = [data.angle_min + data.angle_increment*i for i in range(len(data.ranges))]

def image_scan_trans_callback(data):
    global ranges, angles
    ranges[3] = data.ranges
    angles[3] = data.angles

def init():
    rospy.init_node('scan_plotter', anonymous=True)

    global graphics
    graphics = Graphics()
    graphics.clear()
    graphics.display()

    rospy.Subscriber("/sonar_data", LaserScan, sonar_callback)
    rospy.Subscriber("/sonar_scan", LaserScan, sonar_scan_callback)

    rospy.Subscriber("/image_scan", LaserScan, image_scan_callback)
    rospy.Subscriber("/image_scan_transformed", PlanarData, image_scan_trans_callback)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        draw_stuff()
        r.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass

