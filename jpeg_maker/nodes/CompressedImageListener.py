#!/usr/bin/env python

import roslib; roslib.load_manifest('jpeg_maker')
import rospy

import os

from sensor_msgs.msg import CompressedImage

count = 0

def callback(data):
    global count

    image = data.data
    
    output_file = open('/home/ras/pythonsimpleserver/test' + str(count) + '.jpeg', 'wb')
    
    output_file.write(image)

    output_file.close()

    count += 1

    count = count % 30

def listener():
    rospy.init_node('image_file_writer', anonymous=True)
    rospy.Subscriber('usb_cam0/image_raw/compressed', CompressedImage, callback)

    rospy.spin()

if __name__ == "__main__":
    listener()
