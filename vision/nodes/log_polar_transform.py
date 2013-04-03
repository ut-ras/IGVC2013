#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv, math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

PERSPECTIVE_TRANSFORM = np.array(
[[  2.48253566e+00,   1.65852542e+00,  -5.01512515e+02],
 [  8.35395261e-03,   4.60948006e+00,  -6.44248503e+02],
 [  6.77126073e-05,   4.81987961e-03,   1.00000000e+00]])
def homography_transform(img):
    return cv2.warpPerspective(
            img,
            PERSPECTIVE_TRANSFORM,
            (img.shape[1], img.shape[0])
            )

"""
TODO: THESE TWO CONSTANTS SHOULD BE IN THE PARAMETER SERVER; THEY WILL BE
USED IN WHATEVER NODE PROCESSES THE LOGPOLAR TOPIC
"""
logscale = 20
anglescale = 80

def log_polar_transform(img):
    center = (img.width/2, img.height)
    maxradius = math.sqrt((img.width/2)**2 + (img.height)**2)
    width = int(logscale*math.log(maxradius))

    logpolar = cv.CreateImage(
            (width, anglescale),
            cv.IPL_DEPTH_8U,
            3 # why is this need to be 3 here, but 1 in polar_processor.py?
            )

    cv.LogPolar(
            img,
            logpolar,
            center,
            logscale,
            cv.CV_INTER_LINEAR + cv.CV_WARP_FILL_OUTLIERS
            )

    aligned = cv.GetSubRect(logpolar, (0, anglescale/2, width, anglescale/2))

    return aligned

initWindow = False
def display(homography, log_polar):
    global initWindow
    if not initWindow:
        cv2.namedWindow('homography')
        cv.NamedWindow('log_polar')
        initWindow = True

    cv2.imshow('homography', homography)
    cv.ShowImage('log_polar', log_polar)

curImg = None
def callback(image_data):
    try:
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")

        global curImg
        curImg = input_img
    except CvBridgeError, e:
        print e

def init():
    rospy.init_node('log_polar_transformer')

    sub = rospy.Subscriber('binimg_orange_red_threshold', Image, callback)
    pub = rospy.Publisher('log_polar_transformed', Image)

    r = rospy.Rate(3)

    while not rospy.is_shutdown():
        global curImg

        if curImg != None:
            cv2img = np.asarray(curImg)
            homography = homography_transform(cv2img)
            cvimg_homography = cv.fromarray(homography)
            log_polar = log_polar_transform(cvimg_homography)

            pub.publish(bridge.cv_to_imgmsg(log_polar, encoding="passthrough"))

            # display(homography, log_polar)
            # cv2.waitKey(30)

        r.sleep()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































