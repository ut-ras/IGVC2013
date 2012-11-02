#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import Encoder
from geometry_msgs.msg import Twist 
import serial
import string

pos_pub = rospy.Publisher('enc_data',Encoder)

def publisher():
    ser = serial.Serial(port='/dev/ttyACM1', baudrate = 115200)
    print "PSoC Listener is running on " + ser.portstr
    pos_pub = rospy.Publisher('enc_data', Encoder)
    vel_pub = rospy.Publisher('enc_vel',Twist)
    rospy.init_node('PSoC_Listener')
    ser.write('>ETFM')
    ser.flush()
    while not rospy.is_shutdown():
        line = ser.readline()
        rospy.loginfo("Got Something!: "+ line)
        tokens = string.split(line)
        if(tokens[0] == '(:' and tokens[len(tokens)-1] == ':)'):
            e = Encoder()
            e.left = int(tokens[2])
            e.right = int(tokens[4])
            t = Twist()
            t.linear.x = int(tokens[6])
            t.angular.z = int(tokens[8])
            e.time = long(tokens[10])
            pos_pub.publish(e)
            vel_pub.publish(t)
    ser.write('>DTFM')
    ser.flush()

if __name__ == "__main__":
    publisher()
