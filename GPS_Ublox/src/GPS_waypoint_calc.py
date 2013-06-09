#!/usr/bin/env python

import math
import roslib; roslib.load_manifest('GPS_Ublox')
import rospy

from vn_200_imu.msg import vn_200_gps_soln

RADIUS_OF_EARTH = 6371 * 1000 #meters
#INITIAL_LATITUDE = 42.6781105
#INITIAL_LONGITUDE = -83.1955214

INITIAL_LATITUDE  = 0.0
INITIAL_LONGITUDE = 0.0

COUNT = 0

NUM_TO_BE_AVERAGED = 50
"""
lat_long = [(30.31615, -97.72885),
            (30.31605, -97.72867),
            (30.31588, -97.72884)]
(-22.941460455271443, -1.544497530891049)
(-5.663373082599206, -12.663990195321006)
(-21.98156671340644, -31.56712772497045)

lat_long = [(42.67847595, -83.1955698361),
        (42.67807146, -83.195271733)]
"""

lat_long = []
sub = None

DIST = 1

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

def offset_calc(latitude, longitude):

    global INITIAL_LATITUDE
    global INITIAL_LONGITUDE

    if INITIAL_LONGITUDE == longitude and INITIAL_LATITUDE == latitude:
        x = 0
        y = 0
    else:
        y = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, INITIAL_LONGITUDE, latitude)
        x = haversine_distance(INITIAL_LONGITUDE, INITIAL_LATITUDE, longitude , INITIAL_LATITUDE)

        if longitude < INITIAL_LONGITUDE:
            x = -x
        if latitude < INITIAL_LATITUDE:
            y = -y
    return (x, y)

def deg_min_sec_to_decimal(deg, mins, sec):
    return (deg/abs(deg)) * float(abs(deg) + ((mins + (sec/60.))/60.))

def read_coords():
    global lat_long
    f = open('/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/waypoints', 'r')
    for line in f:
        nums = line.split(' ')

        #print 'mums: ', nums

        lat = deg_min_sec_to_decimal(int(nums[0]), int(nums[1]), float(nums[2].strip()))
        lon = deg_min_sec_to_decimal(int(nums[3]), int(nums[4]), float(nums[5].strip()))
        lat_long += [(lat, lon)]

    #print 'Lat_long = '
    #print lat_long

def gps_callback(data):

    global INITIAL_LONGITUDE
    global INITIAL_LATITUDE
    global COUNT
    global NUM_TO_BE_AVERAGED
    global DIST

    lat = data.latitude
    lon = data.longitude

    INITIAL_LATITUDE  += lat
    INITIAL_LONGITUDE += lon

    COUNT += 1

    if COUNT == NUM_TO_BE_AVERAGED:
        sub.unregister()

        INITIAL_LATITUDE  = float(INITIAL_LATITUDE)/NUM_TO_BE_AVERAGED
        INITIAL_LONGITUDE = float(INITIAL_LONGITUDE)/NUM_TO_BE_AVERAGED

        #print str(INITIAL_LATITUDE), ', ', str(INITIAL_LONGITUDE)

        ref_coords = open('/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/ref_coords', 'w')
        ref_coords.write(str(INITIAL_LATITUDE) + ' ' + str(INITIAL_LONGITUDE))
        ref_coords.close()

        out = open('/home/ras/ros/ros-pkg/IGVC2013/GPS_Ublox/offsets', 'w')

        dance = bool(rospy.get_param('~do_dance'))

        for (lati, longi) in lat_long:
            x, y = offset_calc(lati, longi)
            if dance:
                out.write(str(x+DIST) + ' ' + str(y+DIST) + ' D\n')
                out.write(str(x+DIST) + ' ' + str(y-DIST) + ' D\n')
                out.write(str(x-DIST) + ' ' + str(y-DIST) + ' D\n')
                out.write(str(x-DIST) + ' ' + str(y+DIST) + ' D\n')
            else:
                out.write(str(x) + ' ' + str(y) + ' G\n')

        out.write('0.0 0.0 G\n')

        out.close()
        rospy.logwarn('Done')
        #print offset_calc(lati, longi)
        #print "DONE"


def get_reference_point():
    global sub
    rospy.init_node('waypoint_offset_calculator')
    sub = rospy.Subscriber('vn_200_gps_soln', vn_200_gps_soln, gps_callback)
    rospy.loginfo('Initialized node and subscribed to topic')

    rospy.spin()

if __name__ == "__main__":
    try:
        read_coords()
        get_reference_point()
    except rospy.ROSInterruptException:
        pass
