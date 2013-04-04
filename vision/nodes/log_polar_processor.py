#!/usr/bin/env python
"""
This node listens to the topic that published a log-polar transform of the
perspective corrected image from the camera and tracing through it to output
an emulated LaserScan (in meters) to indicate distance to obstacles
"""

import roslib; roslib.load_manifest('vision')
import rospy, sys, math, pygame
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, LaserScan

SHOW_GRAPHICS = True

bridge = CvBridge()

"""
TODO: THESE TWO CONSTANTS SHOULD BE IN THE PARAMETER SERVER; THEY ARE USED
HERE AND THE NODE THAT PUBLISHES TO THE LOGPOLAR TOPIC
"""
logscale = 20
anglescale = 80

# after the image is homography (perspective) transformed, this is the number
# of pixels that go into one meter
PIXELS_PER_METER = 281.0

# this is the distance in meters from the place on the ground that is seen
# on the bottom of the image to the point on the ground just below the
# front of the robot
DISTANCE_FROM_FRONT = .31

def makeScanFromImg(img):
    rowHeight = 1
    num_planar_rays = img.shape[0]/rowHeight
    maxdist = img.shape[1]
    angleWidth = math.pi/float(num_planar_rays)

    ranges = []

    distance = None
    for angleIndex in range(num_planar_rays):
        angle = math.pi - angleIndex*angleWidth
        row = angleIndex*rowHeight

        for x in range(0, maxdist):
            flag = False
            for y in range(int(row), int(row + rowHeight)):
                if img[y,x] > 0:
                    flag = True
                    break
            if flag:
                break

        distance = math.exp(x/float(logscale))
        distance = distance/PIXELS_PER_METER + DISTANCE_FROM_FRONT

        ranges.append(distance)

    ranges.append(distance)
    ranges = ranges[::-1]

    scan = LaserScan()
    scan.angle_increment = angleWidth
    scan.ranges = ranges

    return scan


SIZEX = 400
SIZEY = 400

refPos = (SIZEX/2, SIZEY/2)
initWindow = False
window = None
background = None
def display(scan):
    global initWindow, window, background

    if not initWindow:
        pygame.init()
        window = pygame.display.set_mode((SIZEX, SIZEY))

        background = pygame.Surface(window.get_size())
        background = background.convert()
        background.fill((0, 0, 0))

        initWindow = True

    window.blit(background, (0,0))
    numRanges = len(scan.ranges)

    for i in range(numRanges):
        dist = scan.ranges[i]
        angle = i*scan.angle_increment

        x = int(refPos[0] + PIXELS_PER_METER*dist*math.cos(angle))
        y = int(refPos[1] + PIXELS_PER_METER*dist*math.sin(angle))

        pygame.draw.line(
                window,
                (255, 255, 255),
                refPos,
                (x, SIZEY - y)
                )

    pygame.display.flip()


curImgData = None
def callback(image_data):
    try:
        global curImgData
        curImgData = np.asarray(bridge.imgmsg_to_cv(image_data, "mono8"))
    except CvBridgeError, e:
        print e

def init():
    rospy.init_node('log_polar_processor')

    sub = rospy.Subscriber('log_polar_transformed', Image, callback)
    pub = rospy.Publisher('image_scan', LaserScan)

    r = rospy.Rate(2)

    while not rospy.is_shutdown():
        global curImgData

        if curImgData != None:
            scan = makeScanFromImg(curImgData)
            pub.publish(scan)

            if SHOW_GRAPHICS:
                display(scan)

        r.sleep()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































