#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threshold_finder import ThresholdFinder

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()
sub = None

scale = .5
rot_mat = cv2.getRotationMatrix2D((0,0), 0, scale) 

def scaleDown(img):
    rows,cols = img.shape[:2]
    scaled_img = cv2.warpAffine(img, rot_mat, (int(cols*scale), int(rows*scale)))
    return scaled_img


def callback(image_data):
    try:
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")
    except CvBridgeError, e:
        print e
        return

    img = scaleDown(np.asarray(input_img))

    global sub
    sub.unregister()

    rospy.loginfo("got an image, sending it off")

    ThresholdFinder.findThreshold(img)
    

def init():
    rospy.init_node('rgb_threshold_finder')
    rospy.loginfo("ready & waiting for usb_cam/image_raw to give us something!")

    global sub
    sub = rospy.Subscriber('usb_cam/image_raw', Image, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































