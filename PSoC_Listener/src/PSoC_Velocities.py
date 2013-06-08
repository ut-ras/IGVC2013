#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy, time
from PSoC_Listener.msg import PSoC
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Thread
import atexit

pub_data = rospy.Publisher('vel_data', Twist)
pub_cmd = rospy.Publisher('psoc_cmd', String)
rate = 20 #milliseconds between messages
LEDTime = 0

@atexit.register
def onExit():
    pub_cmd.publish(String(">LEDR:0"))
    print "PSoC Velocities killed (Turning LED off)"

def sub_LED():
    global LEDTime
    while not rospy.is_shutdown():
        LEDTime = LEDTime - 1
        if LEDTime is -2:
            LEDTime = -1
        if LEDTime is 0:
            pub_cmd.publish(String(">LEDR:100"))
        time.sleep(.1) #run at 10hz

def load_LED():
    global LEDTime
    if LEDTime <= 0:
        pub_cmd.publish(String(">LEDR:5"))
    LEDTime = 10

def dataCallback(data):
    p = Twist()
    rate = data.rate;
    p.linear.x = float(data.vel_v) * .00004871 * 1000 / rate
    #Each raw v-velocity tick = 48.71 um per (time interval between messages)
    p.angular.z = - float(data.vel_w) * .00019482 * 1000 / rate
    #Each raw w-velocity tick = 194.82 urad per (time interval between messages)
    pub_data.publish(p)

def cmdCallback(data):
    p = String()
    p.data = ">SVLX:"+str(int(data.linear.x * rate / .00004871 / 1000 / 2))
    #Each raw v-velocity tick = 48.71 um per (time interval between messages)
    pub_cmd.publish(p)
    p.data = ">SVAZ:"+str(int(-data.angular.z * rate / .00019482 / 1000/ 2))
    #Each raw w-velocity tick = 194.82 urad per (time interval between messages)
    pub_cmd.publish(p)
    load_LED()

def velocities():
    """Conversions between encoder outputs to meters per second"""
    rospy.init_node('PSoC_Velocities')
    sub = rospy.Subscriber('psoc_data', PSoC, dataCallback)
    sub = rospy.Subscriber('vel_cmd', Twist, cmdCallback)
    pub_cmd.publish(String(">LEDR:100"))
    t = Thread(target=sub_LED, args=[])
    t.start()
    print "PSoC Velocities is running"
    rospy.spin()

if __name__ == "__main__":
    try:
        velocities()
    except rospy.ROSInterruptException: pass
