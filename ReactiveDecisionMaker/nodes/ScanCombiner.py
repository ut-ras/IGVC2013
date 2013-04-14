#!/usr/bin/env python
import roslib; roslib.load_manifest('ReactiveDecisionMaker')
import rospy
from sensor_msgs.msg import LaserScan
from ReactiveDecisionMaker.msg import PlanarData

pub = None
sonar_scan = LaserScan()
imgray_data = PlanarData()
"""
imgray_scan = LaserScan()

def publish_scans():
    global sonar_scan, imgray_scan, pub
    data = PlanarData()

    angles = []
    ranges = []

    sonar_len = len(sonar_scan.ranges)
    imgray_len = len(imgray_scan.ranges)

    i = 0
    j = 0

    data.startAngle = min(sonar_scan.angle_min, imgray_scan.angle_min)
    endAngle = max(sonar_scan.angle_max, imgray_scan.angle_max)
    data.angleRange = endAngle - data.startAngle

    while i < sonar_len or j < imgray_len:
        add_imgray = (j < imgray_len)

        if i < sonar_len and j < imgray_len:
            # figure out which has the smaller angle
            imgray_angle = imgray_scan.angle_min + imgray_scan.angle_increment*j
            sonar_angle = sonar_scan.angle_min + sonar_scan.angle_increment*i

            if imgray_angle > sonar_angle:
                add_imgray = False

        if add_imgray:
            # add imgray
            imgray_angle = imgray_scan.angle_min + imgray_scan.angle_increment*j
            angles.append(imgray_angle)
            ranges.append(imgray_scan.ranges[j])
            j += 1
        else:
            # add sonar
            sonar_angle = sonar_scan.angle_min + sonar_scan.angle_increment*i
            angles.append(sonar_angle)
            ranges.append(sonar_scan.ranges[i])
            i += 1

    data.angles = angles
    data.ranges = ranges

    pub.publish(data)

def image_scan_callback(data):
    global imgray_scan
    imgray_scan = data
    publish_scans()
"""
def publish_scans():
    global sonar_scan, imgray_data, pub
    data = PlanarData()

    angles = []
    ranges = []

    sonar_len = len(sonar_scan.ranges)
    imgray_len = len(imgray_data.ranges)

    i = 0
    j = 0

    data.startAngle = min(sonar_scan.angle_min, imgray_data.startAngle)
    endAngle = max(sonar_scan.angle_max, imgray_data.startAngle + imgray_data.angleRange)
    data.angleRange = endAngle - data.startAngle

    while i < sonar_len or j < imgray_len:
        add_imgray = (j < imgray_len)

        if i < sonar_len and j < imgray_len:
            # figure out which has the smaller angle
            imgray_angle = imgray_data.angles[j]
            sonar_angle = sonar_scan.angle_min + sonar_scan.angle_increment*i

            if imgray_angle > sonar_angle:
                add_imgray = False

        if add_imgray:
            # add imgray
            imgray_angle = imgray_data.angles[j]
            angles.append(imgray_angle)
            ranges.append(imgray_data.ranges[j])
            j += 1
        else:
            # add sonar
            sonar_angle = sonar_scan.angle_min + sonar_scan.angle_increment*i
            angles.append(sonar_angle)
            ranges.append(sonar_scan.ranges[i])
            i += 1

    data.angles = angles
    data.ranges = ranges

    pub.publish(data)

def image_scan_trans_callback(data):
    global imgray_data
    imgray_data = data
    publish_scans()

def sonar_scan_callback(data):
    global sonar_scan
    sonar_scan = data
    publish_scans()

def init_subscriptions():
    global pub
    pub = rospy.Publisher("planar_data", PlanarData)

    # sub1 = rospy.Subscriber("image_scan", LaserScan, image_scan_callback)
    sub1 = rospy.Subscriber("image_scan_transformed", PlanarData, image_scan_trans_callback)
    sub2 = rospy.Subscriber("sonar_scan", LaserScan, sonar_scan_callback)

if __name__ == "__main__":
    rospy.init_node('ScanCombiner')
    init_subscriptions()
    rospy.loginfo("we've subscribed to the scan topics!");
    rospy.spin()
