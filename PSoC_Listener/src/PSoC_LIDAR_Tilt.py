#!/usr/bin/env python
import roslib; roslib.load_manifest('PSoC_Listener')
import rospy
from PSoC_Listener.msg import PSoC
from std_msgs.msg import Float32

__LIDAR_TILT_MULTI__ = 1
__LIDAR_TILT_CENTER__ = 0
pub = rospy.Publisher('lidar_tilt', Float32)
def callback(data):
    p = Float32()
    p.data = (data.adc - __LIDAR_TILT_CENTER__) * __LIDAR_TILT_MUTLI__
    pub.publish(p)

def translate():
    rospy.init_node('LIDAR_tilt_angle')
    sub = rospy.Subscriber('psoc_data',PSoC,callback)
    rospy.spin()

if __name__ is '__main__':
    try:
        translate()
    except rospy.ROSInterruptException: pass

