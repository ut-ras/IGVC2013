#!/usr/bin/env python
import roslib; roslib.load_manifest('GPS_Ublox')
import rospy
import serial
import string
from geometry_msgs.msg import Point

ser = serial.Serial(port='/dev/ttyACM1', baudrate = 57600)

def gps():
	rospy.loginfo( "GPS Listener is running on " + ser.portstr )
	pub = rospy.Publisher('gps_data', Point)
	rospy.init_node('GPS_Listener')
	while not rospy.is_shutdown():
		line = ser.readline()
		tokens = line.split(',')
		if(tokens[0] == '$GPRMC'):
			rospy.loginfo( "gps_data/RMC: " + line )
		elif(tokens[0] == '$GPVTG'):
			rospy.loginfo( "gps_data/VTG: " + line )
		elif(tokens[0] == '$GPGGA'):	
			rospy.loginfo( "gps_data/GGA: " + line )
		elif(tokens[0] == '$GPGSA'):
			rospy.loginfo( "gps_data/GSA: " + line )
		elif(tokens[0] == '$GPGSV'):
			rospy.loginfo( "gps_data/GSV: " + line )
		elif(tokens[0] == '$GPGLL'):
			if(tokens[6] == 'A'):
				p = Point()
				p.x = float(tokens[1])
				p.y = float(tokens[3])
				rospy.loginfo( "gps_data/GLL/Latitude: " + tokens[1] )
				rospy.loginfo( "gps_data/GLL/N: " + tokens[2] )
				rospy.loginfo( "gps_data/GLL/Longitude: " + tokens[3] )
				rospy.loginfo( "gps_data/GLL/E: " + tokens[4] )
				rospy.loginfo( "gps_data/GLL/UTC_Time: " + tokens[5] )
				rospy.loginfo( "gps_data/GLL/Valid: " + tokens[6] )
				rospy.loginfo( "gps_data/GLL/Mode&Checksum: " + tokens[7] )
				pub.publish(p)
			else:
				rospy.loginfo("Invaid Data" +line)		
		elif(tokens[0] == '$GPZDA'):
			rospy.loginfo( "gps_data/ZDA: " + line )
		else:
			rospy.loginfo("Command error" +line)
if __name__ == "__main__":
	try:
		gps()
	except rospy.ROSInterruptException: pass 

	
