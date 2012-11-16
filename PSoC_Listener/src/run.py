#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC 
from std_msgs.msg import String
import serial
import string
import atexit
from geometry_msgs.msg import Twist

ser = serial.Serial(port='/dev/ttyACM1', baudrate = 921600)

@atexit.register
def onExit():
    ser.write('>DTFM\n')
    ser.flush()

def callback(data):
    rospy.logdebug('Sending message to PSoC: '+data.data)
    if(data.data[len(data.data)-1]!='\n' and data.data[len(data.data)-1]!='\r'):
        ser.write(data.data+'\n')
    else:
        ser.write(data.data)
    ser.flushOutput()

def psoc():
    rospy.loginfo( "PSoC Listener is running on " + ser.portstr )
    pub = rospy.Publisher('psoc_data', PSoC)
    pubt = rospy.Publisher('sphero_data', Twist)
    rospy.init_node('PSoC_Listener')
    sub = rospy.Subscriber('psoc_cmd', String, callback)
    ser.write('>ETFM\n')
    ser.flushOutput()
    while not rospy.is_shutdown():
        ser.flushInput()
        line = ser.readline()
        tokens = string.split(line)
        if(tokens[0] == '(:' and tokens[len(tokens)-1] == ':)'):
            p = PSoC()
            t = Twist()
            p.left_enc = int(tokens[1])
            p.right_enc = int(tokens[2])
            p.vel_v = int(tokens[3])
            p.vel_w = int(tokens[4])
            t.angular.z = int(tokens[5])
            t.angular.y = int(tokens[6])
            t.angular.x = int(tokens[7])
            p.time = long(tokens[8])
            p.rate = int(tokens[9])
            pub.publish(p)
            pubt.publish(t)
            rospy.logdebug('Telemetry message: '+line)
        else:
            rospy.loginfo('Info message from PSoC: '+ line)
    
if __name__ == "__main__":
    try:
        psoc()
    except rospy.ROSInterruptException: pass
