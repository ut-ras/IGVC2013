#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from std_msgs.msg import String
import serial
import string
import atexit
import time

port = rospy.get_param("port",'/dev/PSoC')
ser = serial.Serial()

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
    global ser
    rospy.init_node('PSoC_Listener')
    pub = rospy.Publisher('psoc_data', PSoC)
    sub = rospy.Subscriber('psoc_cmd', String, callback)
    while not rospy.is_shutdown():
        try:
            ser = serial.Serial(port = port, baudrate = 921600)
            ser.flush()
            ser.write('>ETFM\n')
            ser.flushOutput()
            rospy.loginfo( "PSoC Listener is running on " + ser.portstr )
            while not rospy.is_shutdown():
                ser.flushInput()
                line = ser.readline()
                tokens = string.split(line)
                if(tokens[0] == '(:' and tokens[len(tokens)-1] == ':)'):
                    try:
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
                    except rospy.exceptions.ROSSerializationException as e:
                        rospy.logwarn('PSoC Listener fucked up.')
                        rospy.logwarn(e.message)
                        rospy.logwarn(e.strerror)
                else:
                    rospy.loginfo('Info message from PSoC: '+ line)
        except serial.serialutil.SerialException:
            print "Disconnected... Reconnecting..."
            time.sleep(1)
        except OSError as e:
            print "Shit's really fucked..."
            print e.message,e.strerror
        #finally:
        #    ser.close()
if __name__ == "__main__":
    try:
        psoc()
    except rospy.ROSInterruptException: pass
