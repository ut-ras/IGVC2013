#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import homography_transform, log_polar_transform

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()
pub = None

initWindow = False
def display(homography, log_polar):
    global initWindow
    if not initWindow:
        cv2.namedWindow('homography')
        cv.NamedWindow('log_polar')
        initWindow = True

    cv2.imshow('homography', homography)
    cv.ShowImage('log_polar', log_polar)

def callback(image_data):
    try:
        global bridge
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")
        cv2img = np.asarray(input_img)
        homography = homography_transform(cv2img)
        cvimg_homography = cv.fromarray(homography)
        log_polar = log_polar_transform(cvimg_homography)

        global pub
        pub.publish(bridge.cv_to_imgmsg(log_polar, encoding="passthrough"))

        # display(homography, log_polar)
        # cv2.waitKey(30)
    except CvBridgeError, e:
        print e

def init():
    rospy.init_node('log_polar_transformer')

    global pub
    pub = rospy.Publisher('log_polar_transformed', Image)
    sub = rospy.Subscriber('binimg_orange_red_threshold', Image, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































