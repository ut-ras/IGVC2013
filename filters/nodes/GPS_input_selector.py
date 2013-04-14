#!/usr/bin/env python

import roslib; roslib.load_manifest('GPS_Ublox')
import rospy

from GPS_Ublox.msg  import gps_raw
from vn_200_imu.msg import vn_200_gps_soln, vn_200_ins_soln

NUM_SATELLITES = 0

PUBLISH_INS = False
PUBLISH_GPS = False

def gps_callback(data):
    global NUM_SATELLITES
    global PUBLISH_GPS

    NUM_SATELLITES = data.num_satellites

    msg = gps_raw()

    msg.header         = data.header

    msg.num_satellites = data.num_satellites

    msg.latitude       = data.latitude
    msg.longitude      = data.longitude
    msg.altitude       = data.altitude

    msg.error_present  = data.error_present
    msg.error_string   = data.error_string

    if PUBLISH_GPS:
        pub = rospy.Publisher('gps_data', gps_raw)
        pub.publish(msg)

def ins_callback(data):
    global NUM_SATELLITES
    global PUBLISH_INS

    msg = gps_raw()

    msg.header         = data.header

    msg.num_satellites = NUM_SATELLITES

    msg.latitude       = data.geodetic_latitude
    msg.longitude      = data.geodetic_longitude
    msg.altitude       = data.altitude

    msg.error_present  = data.error_present
    msg.error_string   = data.error_string

    if PUBLISH_INS:
        pub = rospy.Publisher('gps_data', gps_raw)
        pub.publish(msg)

def echoer():
    global PUBLISH_INS
    global PUBLISH_GPS

    rospy.init_node('GPS_input_selector', anonymous=False)
    rospy.loginfo("GPS_input_selector node initialized")

    subscribe_to = str(rospy.get_param('~source'))

    PUBLISH_INS = subscribe_to.lower() == 'ins'
    PUBLISH_GPS = subscribe_to.lower() == 'gps'
    if not PUBLISH_GPS and not PUBLISH_INS:
        rospy.logerror('Invalid input: ' + subscribe_to + '. Enter either ins or gps.')
    else:
        rospy.Subscriber('/vn_200_ins_soln', vn_200_ins_soln, ins_callback)
        rospy.Subscriber('/vn_200_gps_soln', vn_200_gps_soln, gps_callback)
        rospy.loginfo('Subscribe to ins and gps. INS: ' + str(PUBLISH_INS) + " GPS: " + str(PUBLISH_GPS))
    rospy.spin();

if __name__ == '__main__':
    try:
        echoer()
    except rospy.ROSInterruptException: pass
