#!/usr/bin/env python
import roslib
import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

Bridge = CvBridge()

pub = rospy.Publisher('imgray', Image)

def callback(data):
    cv_image = None
    try:
        cv_image = Bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
        print e
    imgray = cv2.cvtColor(numpy.array(cv_image),cv2.COLOR_BGR2GRAY)
    try:
        pub.publish(Bridge.cv_to_imgmsg(cv2.cv.fromarray(imgray), "mono8"))
    except CvBridgeError, e:
        print e

def subscribe():
    
    rospy.init_node('imgray')
    rospy.Subscriber("usb_cam/image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        subscribe()
    except rospy.ROSInterruptException:
        pass
