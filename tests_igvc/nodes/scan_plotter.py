#!/usr/bin/env python
import roslib; roslib.load_manifest('tests_igvc')
import rospy, pygame, math

from sensor_msgs.msg import LaserScan
from filters.msg import PlanarData
from Plotter import Graphics

graphics = None

def plot_scan(ranges, angles, color):
    global graphics

    for i in range(len(ranges)):
        r = ranges[i]
        angle = angles[i]
        x = r*math.cos(angle)
        y = r*math.sin(angle)
        # graphics.plot_line(0, 0, x, y, color)

        if i != len(ranges) - 1:
            r2 = ranges[i + 1]
            angle2 = angles[i + 1]
            x2 = r2*math.cos(angle2)
            y2 = r2*math.sin(angle2)
            graphics.plot_line(x, y, x2, y2, color)

ranges = [[]]*6
angles = [[]]*6
seens = [False]*6
colors = [
    (0,0,255),
    (0,255,255),
    (0,255,0),
    (255,0,0),
    (255,255,0),
    (255,0,255)
    ]

def draw_stuff():
    graphics.clear()

    graphics.draw_axis()

    for i in range(len(ranges)):
        plot_scan(ranges[i], angles[i], colors[i])

    graphics.display()

def makePDataCallback(name, index):
    def handler(data):
        if not seens[index]:
            print "saw",name,":",colors[index]
            seens[index] = True

        ranges[index] = data.ranges
        angles[index] = data.angles

    return handler

def makeLidarScanCallback(name, index):
    def handler(data):
        if not seens[index]:
            print "saw",name,":",colors[index]
            seens[index] = True

        ranges[index] = [
            data.range_max if x < data.range_min else x
            for x in data.ranges
            ]
        angles[index] = [
            data.angle_min + data.angle_increment*i
            for i in range(len(data.ranges))
            ]

    return handler

def makeScanCallback(name, index):
    def handler(data):
        if not seens[index]:
            print "saw",name,":",colors[index]
            seens[index] = True

        ranges[index] = data.ranges
        angles[index] = [
            data.angle_min + data.angle_increment*i
            for i in range(len(data.ranges))
            ]

    return handler

class TopicInfo:
    indexCount = 0

    def __init__(self, topicName, isScan, isLidar=False):
        self.topicName = topicName

        if isScan:
            rospy.Subscriber(
                topicName,
                LaserScan,
                makeLidarScanCallback(topicName, TopicInfo.indexCount)
                if isLidar else
                makeScanCallback(topicName, TopicInfo.indexCount)
                )
        else:
            rospy.Subscriber(
                topicName,
                PlanarData,
                makePDataCallback(topicName, TopicInfo.indexCount)
                )

        TopicInfo.indexCount += 1

def init():
    rospy.init_node('scan_plotter', anonymous=True)

    global graphics
    graphics = Graphics(XAXIS_LENGTH=20, YAXIS_LENGTH=20)
    graphics.clear()
    graphics.display()

    topicInfo = [
        TopicInfo("/sonar_data", True),
        TopicInfo("/sonar_scan", True),
        TopicInfo("/image_scan", True),
        TopicInfo("/image_scan_transformed", False),
        TopicInfo("/scan", True, True),
        TopicInfo("/planar_data", False),
        ]

    r = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        draw_stuff()
        r.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass

