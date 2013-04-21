#!/usr/bin/env python

import math

RADIUS_OF_EARTH = 6371 * 1000 #meters
INITIAL_LATITUDE = 30.31616389
INITIAL_LONGITUDE = -97.728611

lat_long = [(30.31615, -97.72885),
            (30.31605, -97.72867),
            (30.31588, -97.72884)]
(-22.941460455271443, -1.544497530891049)
(-5.663373082599206, -12.663990195321006)
(-21.98156671340644, -31.56712772497045)

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

for (lat, lon) in lat_long:
    print offset_calc(lat, lon)
