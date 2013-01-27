#!/usr/bin/env python

import roslib; roslib.load_manifest('GPS_Ublox')
import rospy
import math

from geometry_msgs.msg import Point

INITIAL_LONGITUDE = []
INITIAL_LATITUDE = []

RADIUS_OF_EARTH = 6371 * 1000 #meters

def degree_to_radians(degrees):
    return math.pi * float(degrees)/180

def haversine_distance(lon1, lat1, lon2, lat2):
    dLat = degree_to_radians(lat2 - lat1)
    dLon = degree_to_radians(lon2 - lon1)
    lat1 = degree_to_radians(lat1)
    lat2 = degree_to_radians(lat2)

    a = (math.sin(dLat/2) ** 2) + ((math.sin(dLon/2) ** 2) * math.cos(lat1) * math.cos(lat2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return RADIUS_OF_EARTH * c


def offset_calc(data):
    pub = rospy.Publisher('gps_data', Point)
    msg = Point()

    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE

    if INITIAL_LATITUDE == [] or INITIAL_LONGITUDE == []:
        rospy.loginfo('Setting starting latitude to: ' +
                str(data.y) + " and starting longitude to: " + str(data.x))
        INITIAL_LATITUDE  = float(data.y)
        INITIAL_LONGITUDE = float(data.x)
        msg.x = 0
        msg.y = 0
    else:
        msg.x = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, INITIAL_LONGITUDE, data.y)
        msg.y = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, data.x           , INITIAL_LATITUDE)
    rospy.loginfo('Publishing to topic gps_data:\n' +
            'x: ' + str(msg.x) + '\ny: ' + str(msg.y))
    pub.publish(msg)


def listener():
    rospy.init_node('GPS_offset_calculator', anonymous=False)
    rospy.Subscriber('gps_data_raw', Point, offset_calc)
    rospy.loginfo('GPS_offset_calculator node initialized')
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
