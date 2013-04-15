#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv, math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

RATE = 5

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

PERSPECTIVE_TRANSFORM = np.array(
[[  1.78778569e+00,   9.19775038e-01,  -2.56236007e+02],
 [  2.24321245e-02,   2.98569665e+00,  -3.37687301e+02],
 [  8.04901494e-05,   2.72273026e-03,   1.00000000e+00]])
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

    # sub = rospy.Subscriber('binimg_orange_red_threshold', Image, callback)
    sub = rospy.Subscriber('vision/out', Image, callback)
    pub = rospy.Publisher('log_polar_transformed', Image)

    r = rospy.Rate(RATE)

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































