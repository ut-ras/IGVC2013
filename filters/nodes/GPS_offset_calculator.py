#!/usr/bin/env python

import roslib; roslib.load_manifest('filters')
import rospy
import math

from geometry_msgs.msg import Point
from GPS_Ublox.msg import gps_raw

FILE_NAME = "/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/ref_coords"

INITIAL_LONGITUDE = 0.0
INITIAL_LATITUDE = 0.0

RADIUS_OF_EARTH = 6371 * 1000 #meters

def set_initial_lat_lon(longitude, latitude):
    global INITIAL_LONGITUDE
    global INITIAL_LATITUDE

    INITIAL_LONGITUDE = longitude
    INITIAL_LATITUDE = latitude

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

# TODO LUCAS
# given starting longitude, latitude in radians,
#   and deltax, deltay in meters
# returns (res_lon, res_lat)
def coord_from_coord_and_haversine(lon, lat, deltax, deltay) :
    pass

def offset_calc(data):

    pub = rospy.Publisher('x_y_offseter', Point)

    msg = Point()

    if data.num_satellites < 3:
        return None

    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE

    rospy.loginfo('INIT_LONG: ' + str(INITIAL_LONGITUDE) + ' INIT_LAT: ' + str(INITIAL_LATITUDE))
    rospy.loginfo('data_long: ' + str(data.longitude) + ' data_lat: ' + str(data.latitude))

    if INITIAL_LONGITUDE == data.longitude and INITIAL_LATITUDE == data.latitude:
        msg.x = 0
        msg.y = 0
    else:
        msg.y = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, INITIAL_LONGITUDE, data.latitude)
        msg.x = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, data.longitude , INITIAL_LATITUDE)

        if data.longitude < INITIAL_LONGITUDE:
            msg.x = -msg.x
        if data.latitude < INITIAL_LATITUDE:
            msg.y = -msg.y

    rospy.loginfo('Publishing to topic gps_data:\n' +
            'x: ' + str(msg.x) + '\ny: ' + str(msg.y) + '\n\n')
    pub.publish(msg)


def listener():
    rospy.init_node('GPS_offset_calculator', anonymous=False)
    #rospy.Subscriber('gps_data_raw', GPS_UBlox_raw, offset_calc)
    rospy.loginfo('GPS_offset_calculator node initialized')

    rospy.Subscriber('/gps_data', gps_raw, offset_calc)
    rospy.spin()

def readInitialLatLon():
    global INITIAL_LONGITUDE, INITIAL_LATITUDE

    try:
        f = open(FILE_NAME, 'r')
    except:
        print 'no file found named '+ FILE_NAME
        return False

    try:
        s = f.readline().split()
        INITIAL_LONGITUDE = float(s[1])
        INITIAL_LATITUDE = float(s[0])
    except:
        print 'couldnt read file'
        return False

    print 'read log,lat: '+str(INITIAL_LONGITUDE)+","+str(INITIAL_LATITUDE)

    return True

if __name__ == '__main__':
    sucess = readInitialLatLon()
    if not sucess:
        print 'failed to read file, quiting'

    if sucess:
        try:
            # set_initial_lat_lon(float(rospy.get_param('/lon')), float(rospy.get_param('/lat')))
            listener()
        except rospy.ROSInterruptException: pass

