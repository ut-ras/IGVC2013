#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from std_msgs.msg import String
import serial
import string
import atexit

ser = serial.Serial(port = rospy.get_param("port",'/dev/PSoC'), baudrate = 921600)

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
    rospy.init_node('PSoC_Listener')
    rospy.loginfo( "PSoC Listener is running on " + ser.portstr )
    pub = rospy.Publisher('psoc_data', PSoC)
    sub = rospy.Subscriber('psoc_cmd', String, callback)
    ser.write('>ETFM\n')
    ser.flushOutput()
    while not rospy.is_shutdown():
        ser.flushInput()
        line = ser.readline()
        tokens = string.split(line)
        if(tokens[0] == '(:' and tokens[len(tokens)-1] == ':)'):
            p = PSoC()
            p.left_enc = int(tokens[1])
            p.right_enc = int(tokens[2])
            p.vel_v = int(tokens[3])
            p.vel_w = int(tokens[4])
            p.adc = int(tokens[5])
            p.time = long(tokens[6])
            p.rate = int(tokens[7])
            pub.publish(p)
            rospy.logdebug('Telemetry message: '+line)
        else:
            rospy.loginfo('Info message from PSoC: '+ line)

if __name__ == "__main__":
    try:
        psoc()
    except rospy.ROSInterruptException: pass
