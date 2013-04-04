#!/usr/bin/env python
import roslib; roslib.load_manifest('lidar_proc')
import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from filters.msg import EKFData

pub = rospy.Publisher('point_cloud_raw', PointCloud)
pub_acc = rospy.Publisher('point_cloud_raw_acc', PointCloud)
angle = Float64()
ekf = EKFData()
MAX_DISTANCE = 4
p_acc = PointCloud()
p_acc.points = []
p.header.frame_id = "map"
def angleCallback(data):
    global angle
    angle = data

def ekfCallback(data):
    global ekf
    ekf = data

def lidarCallback(data):
    global angle
    global ekf
    global p_acc
    p = PointCloud()
    p.points = []
    p.header = data.header
    p.header.frame_id = "map"
    p_acc.header.stamp = data.header.stamp
    p_acc.header.seq = data.header.seq
    current_angle = data.angle_min
    pitch = angle.data + ekf.pitch
    roll = ekf.roll
    for i in data.ranges:
        if i < MAX_DISTANCE:
            point = Point32()
            yaw = current_angle+ekf.yaw
            point.x = i * math.cos(yaw) * math.cos(pitch)
            point.y = i * math.sin(yaw) * math.cos(pitch)
            current_angle += data.angle_increment
            point.z = i * math.sin(pitch)
            p.points += [point]
    p_acc.points += p.points
    pub.publish(p)
    pub_acc.publish(p_acc)

def lidar():
    rospy.init_node('point_cloud_raw', anonymous=False)
    rospy.Subscriber("scan", LaserScan, lidarCallback)
    rospy.Subscriber("hokuyo_angle", Float64, angleCallback)
    rospy.Subscriber("ekf_data", EKFData, ekfCallback)
    rospy.spin()

if __name__ == '__main__':
    lidar()
