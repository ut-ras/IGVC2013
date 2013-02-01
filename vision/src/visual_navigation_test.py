#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy, sys, cv, cv2, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from copy import copy
from math import pi
class video_processor:
    def __init__(self):
        self.sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('heading', Twist)
        self.speed = float(1)
        self.bridge = CvBridge()
        cv.NamedWindow("Input Video")
        #cv.NamedWindow("Blur Video")
        #cv.NamedWindow("HSV Video")
        #cv.NamedWindow("Hue Video")
        #cv.NamedWindow("Saturation Video")
        #cv.NamedWindow("Value Video")
        #cv.NamedWindow("Red-Orange Video")
        #cv.NamedWindow("White Video")
        cv.NamedWindow("Red-Orange and White Video")
        cv.WaitKey(0)

    def callback(self, image_in):
        try:
            input_image = self.bridge.imgmsg_to_cv(image_in,"bgr8")
        except CvBridgeError, e:
            print e
        cv.ShowImage("Input Video", input_image)

        blur_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
        cv.Smooth(input_image,blur_image,cv.CV_BLUR, 10, 10)
        #cv.ShowImage("Blur Video", proc_image)
        proc_image = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC3)
        cv.CvtColor(blur_image, proc_image, cv.CV_BGR2HSV)
        #cv.ShowImage("HSV Video", proc_image)
        split_image = [cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1),cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)]
        cv.Split(proc_image, split_image[0],split_image[1],split_image[2], None )
        #hue = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        #sat = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        #val = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        #cv.Split(proc_image, hue,sat,val, None )
        #cv.ShowImage("Hue Video", hue)
        #cv.ShowImage("Saturation Video", sat)
        #cv.ShowImage("Value Video", val)

        thresh_0 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        thresh_1 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        thresh_2 = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        red_orange = cv.CreateMat(input_image.rows,input_image.cols,cv.CV_8UC1)
        cv.Threshold(split_image[1],thresh_0, 128,255,cv.CV_THRESH_BINARY) # > 50% saturation
        cv.Threshold(split_image[0],thresh_1, 220,255,cv.CV_THRESH_BINARY) # > Purple
        cv.Threshold(split_image[0],thresh_2, 10, 255,cv.CV_THRESH_BINARY_INV) # < Yellow-Orange
        cv.Add(thresh_1,thresh_2,red_orange)
        cv.And(red_orange,thresh_0,red_orange)
        #cv.ShowImage("Red-Orange Video",red_orange)

        cv.CvtColor(blur_image, proc_image, cv.CV_BGR2HLS)
        cv.Split(proc_image, split_image[0], split_image[1],split_image[2], None )
        cv.Threshold(split_image[1],thresh_0, 204,255,cv.CV_THRESH_BINARY) # > 80% Lum
        #cv.ShowImage("White Video",thresh_0)

        cv.WaitKey(1)

        cv.Or(red_orange, thresh_0, thresh_0)
        cv.ShowImage("Red-Orange and White Video",thresh_0)

        ang_z = 0
        x = 0
        for i in range(input_image.rows):
            y = -(input_image.cols / 2)
            row = cv.GetRow(thresh_0,i)
            for j in row.tostring():
                ang_z = ang_z + (x * y *ord(j))
                y = y + 1
            print y
            x = x + 1
        ang_z = (ang_z * pi * 2 * 2 * 4 / 255 / input_image.rows / input_image.rows / input_image.cols / input_image.cols)
        p = Twist()
        p.linear.x = self.speed
        p.angular.z = ang_z
        self.pub.publish(p)

def fuck_it():
    rospy.init_node('visual_navigation_test')
    rospy.loginfo( "Frank is now the vision team. i.e. turning our camera into a really bad line sensor" )
    vp = video_processor()
    rospy.spin()

if __name__ == "__main__":
    try:
        fuck_it()
    except rospy.ROSInterruptException: pass

