#!/usr/bin/env python
import roslib; roslib.load_manifest('filters')
import rospy, math
from sensor_msgs.msg import LaserScan
from filters.msg import PlanarData

pub = None
sonar_scan = LaserScan()
imgray_data = PlanarData()
lidar_scan = LaserScan()

class AngleInfo:
    def __init__(self, min, inc):
        self.min = min
        self.inc = inc

def publish_scans():
    #
    # initialize the data message
    #
    data = PlanarData()

    data.startAngle = min(
        sonar_scan.angle_min,
        imgray_data.startAngle,
        lidar_scan.angle_min
        )

    endAngle = max(
        sonar_scan.angle_max,
        imgray_data.startAngle + imgray_data.angleRange,
        lidar_scan.angle_max
        )

    data.angleRange = endAngle - data.startAngle

    #
    # merge the three scans into one
    #
    angles = []
    ranges = []

    angleInfo = [
        AngleInfo(sonar_scan.angle_min, sonar_scan.angle_increment),
        imgray_data.angles,
        AngleInfo(lidar_scan.angle_min, lidar_scan.angle_increment),
        ]

    rangeArr = [
        sonar_scan.ranges,
        imgray_data.ranges,
        lidar_scan.ranges
        ]

    numTopics = len(angleInfo)

    shouldCalc = [isinstance(x, AngleInfo) for x in angleInfo]
    lengthArr = [len(arr) for arr in rangeArr]
    indexes = [0]*numTopics

    indexArr = range(numTopics)
    lengthsSum = sum(lengthArr)

    while sum(indexes) < lengthsSum:
        minAngle = float("inf")
        bestIndex = None
        bestRange = None

        for i in indexArr:
            index = indexes[i]

            if index < lengthArr[i]:
                if shouldCalc[i]:
                    angle = angleInfo[i].min + angleInfo[i].inc*index
                else:
                    angle = angleInfo[i][index]

                if angle < minAngle:
                    minAngle = angle
                    bestIndex = i
                    bestRange = rangeArr[i][index]

        indexes[bestIndex] += 1
        angles.append(minAngle)
        ranges.append(bestRange)

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

def lidar_scan_callback(data):
    global lidar_scan

    ranges = []

    for i in range(len(data.ranges)):
        angle = data.angle_min + data.angle_increment*i

        if data.ranges[i] <= data.range_min:
            ranges.append(data.range_max)
            """
        elif angle < -1.1 and angle > -1.36:
            ranges.append(data.range_max)
        elif angle < -math.pi/2 + math.pi/6:
            if data.ranges[i] < 1:
                print angle, data.ranges[i]
            ranges.append(data.range_max)
            """
        else:
            ranges.append(data.ranges[i])

    lidar_scan = data
    lidar_scan.ranges = ranges

    publish_scans()

def init_subscriptions():
    global pub
    pub = rospy.Publisher("planar_data", PlanarData)

    sub1 = rospy.Subscriber(
        "image_scan_transformed",
        PlanarData,
        image_scan_trans_callback
        )

    sub2 = rospy.Subscriber("sonar_scan", LaserScan, sonar_scan_callback)
    sub3 = rospy.Subscriber("scan", LaserScan, lidar_scan_callback)

if __name__ == "__main__":
    rospy.init_node('ScanCombiner')
    init_subscriptions()
    rospy.loginfo("we've subscribed to the scan topics!");
    rospy.spin()
