#!/usr/bin/env python
import roslib; roslib.load_manifest('lidar_proc')
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

pub = rospy.Publisher('point_cloud_raw', PointCloud)

def lidarinfo(data):
    p  = PointCloud()
    p.points = []
    p.header = data.header
    current_angle = data.angle_min
    for i in data.ranges:
        point = Point32()
        point.x = i * math.cos(current_angle)
        point.y = i * math.sin(current_angle)
        current_angle += data.angle_increment
        point.z = 0
        p.points += [point]
    pub.publish(p)

def lidar():
    rospy.init_node('point_cloud_raw', anonymous=False)
    rospy.Subscriber("scan", LaserScan, lidarinfo)
    rospy.spin()

if __name__ == '__main__':
    lidar()
