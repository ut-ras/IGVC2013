#!/usr/bin/env python
import roslib; roslib.load_manifest('tests_igvc')
import rospy

from sensor_msgs.msg import LaserScan,Image
from geometry_msgs.msg import Point
from ocean_server_imu.msg import RawData
from PSoC_Listener.msg import PSoC
from um6_imu.msg import UM6IMU
from GPS_Ublox.msg import GPS_UBlox_raw


sensors = { \
        'hokuyo':           {'topic': "scan",              'kind': LaserScan}, \
        'camera':           {'topic': "usb_cam/image_raw", 'kind': Image}, \
        #'ocean server imu': {'topic': "os_imu_data",       'kind': RawData}, \
        'um6 imu':          {'topic': "um6_imu_data",      'kind': UM6IMU}, \
        'gps':              {'topic': "gps_data_raw",      'kind': GPS_UBlox_raw}, \
        'psoc':             {'topic': "psoc_data",         'kind': PSoC} }


def make_callback(sensor_name):
    def callback (data):
        sensors[sensor_name]['seen'] = True
        rospy.loginfo("saw " + sensor_name + " data!")
        sensors[sensor_name]['sub'].unregister()
    return callback;


def listener():
    rospy.init_node('connectivity_tester')

    for name in sensors:
        topic_name = sensors[name]['topic']
        data_type = sensors[name]['kind']
        callback = make_callback(name)
        sensors[name]['sub'] = rospy.Subscriber(topic_name, data_type, callback)
        sensors[name]['seen'] = False

    r = rospy.Rate(1)
    all_seen = True

    while not rospy.is_shutdown():
        all_seen = True

        for name in sensors:
            if not sensors[name]['seen']:
                rospy.loginfo("not seeing " + name + " data...")
                all_seen = False

        if all_seen:
            break;

        r.sleep()

    if all_seen:
        rospy.loginfo("expected sensors: " + ", ".join(sensors.keys()))
        rospy.loginfo("all expected sensor topics seen publishing data!")

if __name__ == '__main__':
    listener()
