#!/usr/bin/env python
import roslib; roslib.load_manifest('SonarArray')
import rospy
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher('sonar_scan', LaserScan)

SONAR_ARRAY_RADIUS = .279

oldRanges = [0]*12
alpha = .6

def callback(data):
    global oldRanges, alpha, SONAR_ARRAY_RADIUS, pub

    scan = LaserScan()
    scan.header.stamp = data.header.stamp
    scan.header.frame_id = data.header.frame_id
    scan.header.seq = data.header.seq
    scan.angle_min = data.angle_min
    scan.angle_max = data.angle_max
    scan.angle_increment = data.angle_increment
    scan.scan_time = data.scan_time
    scan.ranges = [0]*12

    # low-pass filter
    for i in range(len(data.ranges)):
        scan.ranges[i] = alpha*data.ranges[i] + (1 - alpha)*oldRanges[i]
        oldRanges[i] = scan.ranges[i]

        scan.ranges[i] += SONAR_ARRAY_RADIUS

    pub.publish(scan)

def init():
    rospy.init_node('SonarProc')

    global sub
    sub = rospy.Subscriber('sonar_data', LaserScan, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass
