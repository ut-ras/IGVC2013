#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv2, cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

RATE = 5

# for conversion of raw image topic data to OpenCV image data
bridge = CvBridge()

def hsv_orange_red_threshold(input_image):
    blur_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
    cv.Smooth(input_image,blur_image,cv.CV_BLUR, 10, 10)
    proc_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
    cv.CvtColor(blur_image, proc_image, cv.CV_BGR2HSV)
    split_image = [cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)]
    cv.Split(proc_image, split_image[0],split_image[1],split_image[2], None )

    thresh_0 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    thresh_1 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    thresh_2 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    red_orange = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
    cv.Threshold(split_image[1],thresh_0, 128,255,cv.CV_THRESH_BINARY) # > 50% saturation
    cv.Threshold(split_image[0],thresh_1, 220,255,cv.CV_THRESH_BINARY) # > Purple
    cv.Threshold(split_image[0],thresh_2, 10, 255,cv.CV_THRESH_BINARY_INV) # < Yellow-Orange
    cv.Add(thresh_1,thresh_2,red_orange)
    cv.And(red_orange,thresh_0,red_orange)

    return red_orange

initWindow = False
def display(img):
    global initWindow
    if not initWindow:
        cv.NamedWindow('result')
        initWindow = True

    cv.ShowImage("result", img)

curImg = None
def callback(image_data):
    try:
        input_img = bridge.imgmsg_to_cv(image_data, "bgr8")

        global curImg
        curImg = input_img
    except CvBridgeError, e:
        print e

def init():
    rospy.init_node('orange_red_thresholder')

    sub = rospy.Subscriber('usb_cam/image_raw', Image, callback)
    pub = rospy.Publisher('binimg_orange_red_threshold', Image)

    r = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        global curImg

        if curImg != None:
            resImg = hsv_orange_red_threshold(curImg)
            pub.publish(bridge.cv_to_imgmsg(resImg, encoding="passthrough"))

            # display(resImg)
            # cv2.waitKey(30)

        r.sleep()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass































