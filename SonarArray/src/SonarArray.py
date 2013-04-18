#!/usr/bin/env python
import roslib; roslib.load_manifest('SonarArray')
import rospy
from sensor_msgs.msg import LaserScan
import serial
import string
from math import pi
import time

port = '/dev/ttyACM1'


MAX_VALUE = float(4.5)
def Process(inp):
    if inp == 'Error':
        return MAX_VALUE
    else:
        try:
            return float(inp)/5600
        except ValueError as e:
            return MAX_VALUE

def sonar():
    global port
    rospy.init_node('Sonar_Array')
    rospy.loginfo( "Sonar Array serial connection is running on " + port)
    pub = rospy.Publisher('sonar_data',LaserScan)
    sequence = 0
    while not rospy.is_shutdown():
        try:
            with serial.Serial(port = port, baudrate = 115200) as ser:
                while not rospy.is_shutdown():
                    print "Starting..."
                    ser.flushInput()
                    line = ser.readline()
                    print "Got line! ", line
                    tokens = line.split()
                    print "Number of tokens: ", len(tokens)
                    if len(tokens) is 13:
                        print '-'+tokens[0]+'-'
                        if(tokens[0] == 'Sonars:'):
                            p = LaserScan()
                            p.header.stamp = rospy.get_rostime()
                            p.header.frame_id = 'sonar_array_frame'
                            p.header.seq = sequence
                            p.angle_min = -pi/2
                            p.angle_max = pi/2
                            p.angle_increment = pi/11
                            p.scan_time = .1
                            p.ranges = [Process(i) for i in tokens[1:13]]
                            p.ranges.reverse()
                            pub.publish(p)
        except IOError:
            print "Disconnected? (Error)"
        time.sleep(1)

if __name__ == "__main__":
    try:
        sonar()
    except rospy.ROSInterruptException: pass

