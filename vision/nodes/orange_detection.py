#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import hsv_orange_red_threshold

pub = None

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

initWindow = False
def display(img):
    global initWindow
    if not initWindow:
        cv.NamedWindow('result')
        initWindow = True

    cv.ShowImage("result", img)

def callback(image_data):
    try:
        global bridge
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")
        resImg = hsv_orange_red_threshold(input_img)

        global pub
        pub.publish(bridge.cv_to_imgmsg(resImg, encoding="passthrough"))

        # display(resImg)
        # cv2.waitKey(30)
    except CvBridgeError, e:
        print e

def init():
    rospy.init_node('orange_red_thresholder')

    global pub
    pub = rospy.Publisher('binimg_orange_red_threshold', Image)
    sub = rospy.Subscriber('usb_cam/image_raw', Image, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass

