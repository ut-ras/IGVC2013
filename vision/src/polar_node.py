#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from polar_processor import ImageProcessor

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

proc = ImageProcessor()

curImg = None

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

    global curImg
    curImg = scaleDown(np.asarray(input_img))

def init():
    rospy.init_node('polar_node')
    rospy.loginfo("polaring!")

    sub = rospy.Subscriber('usb_cam/image_raw', Image, callback)

    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        global curImg
        if curImg != None:
            proc.calcPolarImage(curImg)
            proc.calcPlanarData()
            proc.draw()
            proc.display()
            cv2.waitKey(30)

        r.sleep()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































