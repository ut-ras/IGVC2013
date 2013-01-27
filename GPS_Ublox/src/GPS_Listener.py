#!/usr/bin/env python
import roslib; roslib.load_manifest('GPS_Ublox')
import rospy
import serial
import string
import math
from geometry_msgs.msg import Point

ser = serial.Serial(port='/dev/U-Blox', baudrate = 57600)

def gps():
    rospy.loginfo( "GPS Listener is running on " + ser.portstr )
    pub = rospy.Publisher('gps_data_raw', Point)
    rospy.init_node('GPS_Listener')
    while not rospy.is_shutdown():
        line = ser.readline()
        tokens = line.split(',')
        if(tokens[0] == '$GPRMC'):
            pass
            #rospy.loginfo( "gps_data/RMC: " + line )
        elif(tokens[0] == '$GPVTG'):
            pass
            #rospy.loginfo( "gps_data/VTG: " + line )
        elif(tokens[0] == '$GPGGA'):
            pass
            #rospy.loginfo( "gps_data/GGA: " + line )
        elif(tokens[0] == '$GPGSA'):
            pass
            #rospy.loginfo( "gps_data/GSA: " + line )
        elif(tokens[0] == '$GPGSV'):
            pass
            #rospy.loginfo( "gps_data/GSV: " + line )
        elif(tokens[0] == '$GPGLL'):
            pass
            if(tokens[6] == 'A'):
                p = Point()
                p.y = (float(tokens[1][:2]) + float(tokens[1][2:])/60)
                p.x = (float(tokens[3][:3]) + float(tokens[3][3:])/60)
                if tokens[4] == 'W':
                    p.x = -p.x
                if tokens[2] == 'S':
                    p.y = -p.y
                #rospy.loginfo( "gps_data/GLL/Latitude: " + tokens[1] )
                #rospy.loginfo( "gps_data/GLL/N: " + tokens[2] )
                #rospy.loginfo( "gps_data/GLL/Longitude: " + tokens[3] )
                #rospy.loginfo( "gps_data/GLL/E: " + tokens[4] )
                #rospy.loginfo( "gps_data/GLL/UTC_Time: " + tokens[5] )
                #rospy.loginfo( "gps_data/GLL/Valid: " + tokens[6] )
                #rospy.loginfo( "gps_data/GLL/Mode&Checksum: " + tokens[7] )

                rospy.loginfo("Publish to topic /gps_data:\n" +
                        "Latitude: " + str(p.y) + "  Longitude: " + str(p.x))

                pub.publish(p)
            else:
                rospy.logerr("Invalid Data recieved! Please check if antenna is connected properly")
        elif(tokens[0] == '$GPZDA'):
            pass
            #rospy.loginfo( "gps_data/ZDA: " + line )
        else:
            rospy.loginfo("Command error" +line)
if __name__ == "__main__":
    try:
        gps()
    except rospy.ROSInterruptException: pass
